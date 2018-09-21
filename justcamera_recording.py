from picamera import PiCamera
import time

camera = PiCamera()
camera.vflip = True

#camera.start_preview()
camera.start_recording('some_video.h264')
time.sleep(120)
camera.stop_recording()
#camera.stop_preview()

