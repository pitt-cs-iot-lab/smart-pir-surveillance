from picamera import PiCamera
import time


class CameraRawRecord:

    def __init__(self, duration, video_name, include_preview=False):
        self.duration = duration
        self.video_name = video_name
        self.include_preview = include_preview
        self.camera = PiCamera()
        self.camera.vflip = True

    def start(self):
        if self.include_preview is True:
            self.camera.start_preview()
            self.camera.start_recording(self.video_name)

            time.sleep(self.duration)

            self.camera.stop_recording()
            self.camera.stop_preview()
