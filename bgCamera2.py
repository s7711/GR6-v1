# bgCamera2.py
#
# Manages the camera in the background. Uses picamera2
#
# Unlike bgCamera.py, this returns an ndarray rather than a jpg.
# The timing isn't perfect, but that could be fixed later.
#
# Usage:
#   import bgCamera2
#   cam = bgCamera2.BgCamera()
#   img = cam.img()  # Get the next image as an ndarray
#   cam.stop()  # Stop the camera thread
#
# Multiple threads can be waiting for the same image

import threading
import time
import numpy as np
import picamera2
import logging

class BgCamera:
    def __init__(self):
        self.picam = picamera2.Picamera2()
        self.config = self.picam.create_still_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
        self.picam.configure(self.config)
        self.picam.start()
        self._cond = threading.Condition()
        self._img = None
        self._img_version = 0 # Used to fix wait() issues
        self._img_timestamp = None # Nanoseconds from CLOCK_MONOTONIC
        self._img_exposure_us = None # Microseconds
        self._running = False
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            request = self.picam.capture_request()
            frame = request.make_array("main")
            metadata = request.get_metadata()
            request.release()

            with self._cond:
                self._img = frame
                self._img_timestamp = metadata.get("Timestamp")  # Nanoseconds from CLOCK_MONOTONIC
                self._img_exposure_us = metadata.get("ExposureTime")  # Microseconds
                self._img_version += 1
                self._cond.notify_all()
            time.sleep(0.2)  # ~5 fps


    def img(self):
        """Wait for the next image. Log if wait() returns spuriously."""
        with self._cond:
            last_version = self._img_version
            while last_version == self._img_version:
                self._cond.wait()
                if last_version == self._img_version:
                    logging.info("Spurious wakeup detected in BgCamera.img()")
        return self._img  # New image is available

    def img2(self):
        """Wait for the next image and return (image, monotonic_time_sec, exposure_time_sec)."""
        with self._cond:
            last_version = self._img_version
            while last_version == self._img_version:
                self._cond.wait()
                if last_version == self._img_version:
                    logging.info("Spurious wakeup detected in BgCamera.img2()")
        
        # Convert units
        mt = self._img_timestamp / 1e9 if self._img_timestamp is not None else None
        ex = self._img_exposure_us / 1e6 if self._img_exposure_us is not None else None
        return self._img, mt, ex


# ...existing code...
    def stop(self):
        self._running = False
        self._thread.join()
        self.picam.stop()