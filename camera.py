import os
import pathlib
import threading
import time
from typing import Generator, Optional

try:
    from picamera2 import Picamera2
    _PICAMERA2_AVAILABLE = True
except ImportError:
    _PICAMERA2_AVAILABLE = False

try:
    import cv2
except ImportError as exc:
    raise RuntimeError(
        "OpenCV (cv2) is required for streaming. Install with "
        "`sudo apt-get install python3-opencv` or `pip install opencv-python`."
    ) from exc

_camera_lock  = threading.Lock()
_camera:  Optional["cv2.VideoCapture"] = None
_picam2:  Optional["Picamera2"]        = None
_camera_index = 0
_using_picam2 = False


def _is_dir_writable(path: pathlib.Path) -> bool:
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
    return candidates[0]


capture_dir = _resolve_capture_dir()


def _init_camera() -> "cv2.VideoCapture":
    global _picam2, _using_picam2
    if _PICAMERA2_AVAILABLE and _picam2 is None:
        _picam2 = Picamera2()
        _picam2.configure(_picam2.create_preview_configuration(main={"size": (640, 480)}))
        _picam2.start()
        _using_picam2 = True
        return None

    cap = cv2.VideoCapture(_camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(_camera_index)
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera. Ensure /dev/video0 exists, no other process is using it, "
            "and the v4l2 driver is loaded (sudo modprobe bcm2835-v4l2)."
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,          30)
    return cap


def ensure_camera() -> "cv2.VideoCapture":
    global _camera
    with _camera_lock:
        if _using_picam2 and _picam2 is not None:
            return _camera
        if _camera is None:
            _camera = _init_camera()
        return _camera


def camera_active() -> bool:
    return _camera is not None or _picam2 is not None


def camera_frames() -> Generator[bytes, None, None]:
    ensure_camera()
    while True:
        with _camera_lock:
            if _picam2 is not None:
                frame = _picam2.capture_array("main")
                if frame is None:
                    success = False
                else:
                    frame   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
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


def capture_frame():
    ensure_camera()
    with _camera_lock:
        if _picam2 is not None:
            frame = _picam2.capture_array("main")
            if frame is None:
                return None
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if _camera is None:
            return None
        success, frame = _camera.read()
        return frame if success else None


def release_camera() -> None:
    global _camera, _picam2, _using_picam2
    with _camera_lock:
        if _camera is not None:
            _camera.release()
            _camera = None
        if _picam2 is not None:
            _picam2.stop()
            _picam2 = None
        _using_picam2 = False
