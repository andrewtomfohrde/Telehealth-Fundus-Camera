import threading
import time
from typing import Optional

import RPi.GPIO as GPIO

STEP_PIN   = 16   # GPIO16
DIR_PIN    = 20   # GPIO20
EN_PIN     = 21   # GPIO21
STEP_DELAY = 0.0005  # seconds between step transitions

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN,  GPIO.OUT)
GPIO.setup(EN_PIN,   GPIO.OUT)

motor_thread:      Optional[threading.Thread] = None
motor_running      = False
current_direction: Optional[str] = None
thread_lock        = threading.Lock()


def enable_motor():
    GPIO.output(EN_PIN, GPIO.LOW)


def disable_motor():
    GPIO.output(EN_PIN, GPIO.HIGH)


def _motor_worker(direction: str, delay: float):
    GPIO.output(DIR_PIN, GPIO.HIGH if direction == "CW" else GPIO.LOW)
    try:
        while motor_running and current_direction == direction:
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(delay / 2)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(delay / 2)
    finally:
        GPIO.output(STEP_PIN, GPIO.LOW)


def start_motor(direction: str, delay: float = STEP_DELAY):
    global motor_thread, motor_running, current_direction
    with thread_lock:
        if motor_running and current_direction == direction:
            return
        if motor_running:
            stop_motor(wait=True)
        enable_motor()
        motor_running     = True
        current_direction = direction
        motor_thread      = threading.Thread(
            target=_motor_worker, args=(direction, delay), daemon=True
        )
        motor_thread.start()


def stop_motor(wait: bool = False):
    global motor_thread, motor_running, current_direction
    with thread_lock:
        motor_running     = False
        current_direction = None
        thread            = motor_thread
        motor_thread      = None
    if wait and thread is not None and thread.is_alive():
        thread.join(timeout=1.0)
    disable_motor()


def cleanup_gpio():
    stop_motor(wait=True)
    GPIO.cleanup()
