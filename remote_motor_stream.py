"""
Single-file Raspberry Pi motor control + live video stream server.


Features:
- MJPEG stream from the Pi camera at /stream
- Simple web UI (/) to view video and send CW/CCW/Stop commands
- JSON API at /api/motor and /api/status
- Motor control using GPIO (same logic as previous MotorControl.py)


Run on the Pi:
   python3 remote_motor_stream.py


From another computer on the same network:
   Open http://<pi-ip>:5000/
"""


import atexit
import os
import pathlib
import threading
import time
from typing import Generator, Optional


from flask import Flask, Response, jsonify, render_template_string, request, send_from_directory


import RPi.GPIO as GPIO


# Prefer Picamera2 (as in PreviewTest). Fallback to OpenCV VideoCapture.
try:
   from picamera2 import Picamera2
   _PICAMERA2_AVAILABLE = True
except ImportError:
   _PICAMERA2_AVAILABLE = False


try:
   import cv2
except ImportError as exc:
   raise RuntimeError(
       "OpenCV (cv2) is required for streaming. Install with `sudo apt-get install python3-opencv` "
       "or `pip install opencv-python`."
   ) from exc


# -----------------------
# Motor setup and helpers
# -----------------------


# Pin configuration
STEP_PIN = 16  # GPIO16
DIR_PIN = 20   # GPIO20
EN_PIN = 21    # GPIO21


STEP_DELAY = 0.0005  # Seconds between step transitions


# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(EN_PIN, GPIO.OUT)


motor_thread: Optional[threading.Thread] = None
motor_running = False
current_direction: Optional[str] = None
thread_lock = threading.Lock()




def enable_motor():
   """Enable stepper driver (active low)."""
   GPIO.output(EN_PIN, GPIO.LOW)




def disable_motor():
   """Disable stepper driver."""
   GPIO.output(EN_PIN, GPIO.HIGH)




def _motor_worker(direction: str, delay: float):
   """Background worker that toggles the step pin while motor_running stays true."""
   GPIO.output(DIR_PIN, GPIO.HIGH if direction == "CW" else GPIO.LOW)
   try:
       while motor_running and current_direction == direction:
           GPIO.output(STEP_PIN, GPIO.HIGH)
           time.sleep(delay / 2)
           GPIO.output(STEP_PIN, GPIO.LOW)
           time.sleep(delay / 2)
   finally:
       # Ensure the motor is not left energized if the thread exits unexpectedly
       GPIO.output(STEP_PIN, GPIO.LOW)




def start_motor(direction: str, delay: float = STEP_DELAY):
   """Start continuous motion in the background thread."""
   global motor_thread, motor_running, current_direction
   with thread_lock:
       if motor_running and current_direction == direction:
           return
       if motor_running:
           stop_motor(wait=True)


       enable_motor()
       motor_running = True
       current_direction = direction
       motor_thread = threading.Thread(
           target=_motor_worker,
           args=(direction, delay),
           daemon=True,
       )
       motor_thread.start()




def stop_motor(wait: bool = False):
   """Signal the worker to stop and optionally wait for it."""
   global motor_thread, motor_running, current_direction
   with thread_lock:
       motor_running = False
       current_direction = None
       thread = motor_thread
       motor_thread = None


   if wait and thread is not None and thread.is_alive():
       thread.join(timeout=1.0)


   disable_motor()




def cleanup_gpio():
   """Release GPIO resources safely."""
   stop_motor(wait=True)
   GPIO.cleanup()




# -----------------------
# Camera + Flask server
# -----------------------


app = Flask(__name__)


_camera_lock = threading.Lock()
_camera: Optional["cv2.VideoCapture"] = None
_picam2: Optional["Picamera2"] = None
_camera_index = 0  # default device index for V4L2 capture
_using_picam2 = False




def _is_dir_writable(path: pathlib.Path) -> bool:
   """Return True when the directory exists and is writable by this process."""
   return path.exists() and path.is_dir() and os.access(path, os.W_OK)




def _resolve_capture_dir() -> pathlib.Path:
   """
   Pick a writable capture directory.


   Priority:
   1. CAPTURE_DIR environment override
   2. captures/ next to this script
   3. ~/Pictures/remote_motor_stream_captures
   4. /tmp/remote_motor_stream_captures
   """
   env_dir = os.environ.get("CAPTURE_DIR")
   if env_dir:
       return pathlib.Path(env_dir).expanduser().resolve()


   candidates = [
       pathlib.Path(__file__).resolve().parent / "captures",
       pathlib.Path.home() / "Pictures" / "remote_motor_stream_captures",
       pathlib.Path("/tmp/remote_motor_stream_captures"),
   ]


   for candidate in candidates:
       try:
           candidate.mkdir(parents=True, exist_ok=True)
       except OSError:
           continue
       if _is_dir_writable(candidate):
           return candidate


   # Fall back to the script directory candidate so the caller gets a precise error.
   return candidates[0]




_capture_dir = _resolve_capture_dir()




def _init_camera() -> "cv2.VideoCapture":
   """Open the camera using Picamera2 as in PreviewTest, else OpenCV V4L2."""
   global _picam2, _using_picam2
   if _PICAMERA2_AVAILABLE and _picam2 is None:
       _picam2 = Picamera2()
       # Mirror PreviewTest configuration
       preview_config = _picam2.create_preview_configuration(main={"size": (640, 480)})
       _picam2.configure(preview_config)
       _picam2.start()
       _using_picam2 = True
       return None  # Picamera2 path returns frames differently


   # Try V4L2 backend first (often needed on Pi with libcamera-v4l2).
   cap = cv2.VideoCapture(_camera_index, cv2.CAP_V4L2)
   if not cap.isOpened():
       # Fallback to default backend
       cap = cv2.VideoCapture(_camera_index)


   if not cap.isOpened():
       raise RuntimeError(
           "Could not open camera. Ensure /dev/video0 exists, no other process is using it, "
           "and the v4l2 driver is loaded (sudo modprobe bcm2835-v4l2)."
       )


   # Best-effort sane defaults (reduce if memory warnings)
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
   cap.set(cv2.CAP_PROP_FPS, 30)
   return cap




def _ensure_camera() -> "cv2.VideoCapture":
   """Initialize the camera lazily if it is not already opened."""
   global _camera
   with _camera_lock:
       if _using_picam2 and _picam2 is not None:
           return _camera
       if _camera is None:
           _camera = _init_camera()
       return _camera




def _camera_frames() -> Generator[bytes, None, None]:
   """Yield JPEG encoded frames for the MJPEG stream."""
   global _camera, _picam2
   _ensure_camera()
   while True:
       with _camera_lock:
           if _picam2 is not None:
               # Picamera2 path (RGB -> convert for JPEG encode)
               frame = _picam2.capture_array("main")
               if frame is None:
                   success = False
               else:
                   frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                   success = True
           else:
               if _camera is None:
                   break
               success, frame = _camera.read()


       if not success:
           time.sleep(0.05)
           continue


       ok, buffer = cv2.imencode(".jpg", frame)
       if not ok:
           continue


       yield (
           b"--frame\r\n"
           b"Content-Type: image/jpeg\r\n\r\n"
           + buffer.tobytes()
           + b"\r\n"
       )




def _release_camera() -> None:
   """Release camera resources safely."""
   global _camera, _picam2, _using_picam2
   with _camera_lock:
       if _camera is not None:
           _camera.release()
           _camera = None
       if _picam2 is not None:
           _picam2.stop()
           _picam2 = None
       _using_picam2 = False




@app.route("/")
def index() -> str:
   """Serve a small control panel page."""
   return render_template_string(
       """
       <!doctype html>
       <html>
       <head>
           <title>Pi Motor + Video</title>
           <style>
               body { font-family: Arial, sans-serif; max-width: 760px; margin: 20px auto; }
               header { display: flex; align-items: center; justify-content: space-between; }
               .controls { margin: 14px 0; display: flex; gap: 10px; }
               button { padding: 12px 16px; font-size: 16px; cursor: pointer; }
               #status { margin-top: 10px; padding: 10px; background: #f1f1f1; }
               img { width: 100%; max-height: 520px; object-fit: contain; background: #000; }
           </style>
       </head>
       <body>
           <header>
               <h2>Live Video + Motor Control</h2>
               <span id="status">Idle</span>
           </header>
           <img id="stream" src="/stream" alt="Video stream unavailable">
           <div class="controls">
               <button onclick="sendCommand('CW')">Clockwise</button>
               <button onclick="sendCommand('STOP')">Stop</button>
               <button onclick="sendCommand('CCW')">Counter-Clockwise</button>
               <button onclick="takePhoto()">Take Photo</button>
           </div>
           <div id="last-photo"></div>
           <script>
               async function sendCommand(direction) {
                   const payload = { direction };
                   try {
                       const res = await fetch("/api/motor", {
                           method: "POST",
                           headers: { "Content-Type": "application/json" },
                           body: JSON.stringify(payload),
                       });
                       const data = await res.json();
                       document.getElementById("status").innerText = data.message || "OK";
                   } catch (err) {
                       document.getElementById("status").innerText = "Error: " + err;
                   }
               }


               async function takePhoto() {
                   try {
                       document.getElementById("status").innerText = "Capturing photo...";
                       const res = await fetch("/api/capture", { method: "POST" });
                       const data = await res.json();
                       if (!res.ok) throw new Error(data.message || res.statusText);
                       document.getElementById("status").innerText = data.message || "Captured";
                       if (data.url) {
                           document.getElementById("last-photo").innerHTML =
                               '<a href="' + data.url + '" target="_blank" rel="noopener">View last photo</a>';
                       }
                   } catch (err) {
                       document.getElementById("status").innerText = "Error: " + err;
                   }
               }
           </script>
       </body>
       </html>
       """
   )




@app.route("/stream")
def stream() -> Response:
   """Return a multipart MJPEG stream."""
   _ensure_camera()
   return Response(
       _camera_frames(),
       mimetype="multipart/x-mixed-replace; boundary=frame",
   )




@app.route("/api/motor", methods=["POST"])
def api_motor() -> Response:
   """Start/stop the motor via JSON payload."""
   payload = request.get_json(silent=True) or {}
   direction = (payload.get("direction") or "").upper()
   delay = float(payload.get("delay", STEP_DELAY))


   if direction not in {"CW", "CCW", "STOP"}:
       return jsonify({"message": "direction must be CW, CCW, or STOP"}), 400


   if direction == "STOP":
       stop_motor()
       message = "Motor stopped"
   else:
       start_motor(direction, delay=delay)
       message = f"Motor running {direction}"


   return jsonify({"message": message, "direction": direction})




@app.route("/api/status")
def api_status() -> Response:
   """Basic health check endpoint."""
   return jsonify({"camera": (_camera is not None) or (_picam2 is not None), "motor_running": motor_running})




@app.route("/snapshot")
def snapshot() -> Response:
   """Return a single JPEG frame for quick debugging."""
   _ensure_camera()
   with _camera_lock:
       if _picam2 is not None:
           frame = _picam2.capture_array("main")
           if frame is None:
               return jsonify({"message": "camera frame unavailable"}), 503
           frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
       else:
           if _camera is None:
               return jsonify({"message": "camera not initialized"}), 503
           success, frame = _camera.read()
           if not success:
               return jsonify({"message": "camera read failed"}), 503


   ok, buffer = cv2.imencode(".jpg", frame)
   if not ok:
       return jsonify({"message": "encode failed"}), 500


   return Response(buffer.tobytes(), mimetype="image/jpeg")




def _capture_frame():
   """Grab a single frame from the active camera."""
   _ensure_camera()
   with _camera_lock:
       if _picam2 is not None:
           frame = _picam2.capture_array("main")
           if frame is None:
               return None
           frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
           return frame
       if _camera is None:
           return None
       success, frame = _camera.read()
       if not success:
           return None
       return frame




@app.route("/api/capture", methods=["POST"])
def api_capture() -> Response:
   """Capture a still image and save it to disk."""
   frame = _capture_frame()
   if frame is None:
       return jsonify({"message": "camera unavailable"}), 503


   try:
       _capture_dir.mkdir(parents=True, exist_ok=True)
   except OSError as exc:
       return jsonify({"message": f"failed to create capture directory: {exc}", "path": str(_capture_dir)}), 500


   filename = f"capture_{int(time.time())}.jpg"
   filepath = _capture_dir / filename
   try:
       wrote_file = cv2.imwrite(str(filepath), frame)
   except Exception as exc:
       return jsonify({"message": f"failed to write file: {exc}", "path": str(filepath)}), 500
   if not wrote_file:
       return jsonify({"message": "failed to write file", "path": str(filepath)}), 500


   return jsonify(
       {
           "message": "Captured image",
           "filename": filename,
           "url": f"/captures/{filename}",
           "path": str(filepath),
       }
   )




@app.route("/captures/<path:filename>")
def get_capture(filename: str):
   """Serve a saved capture file."""
   return send_from_directory(_capture_dir, filename)




def _cleanup() -> None:
   """Ensure resources are released on shutdown."""
   cleanup_gpio()
   _release_camera()




if __name__ == "__main__":
   _ensure_camera()
   atexit.register(_cleanup)
   print("Starting server on http://0.0.0.0:5000")
   try:
       app.run(host="0.0.0.0", port=5000, threaded=True)
   finally:
       _cleanup()



