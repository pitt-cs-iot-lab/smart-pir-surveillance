import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from threading import Thread
from picamera import PiCamera
import RPi.GPIO as GPIO
import Queue
import time
import cv2

debug = 1
pir_pin = 8 #Assign pin 8 to PIR
led_pin = 10 #Assign pin 10 to LED
GPIO.setmode(GPIO.BOARD) #Set GPIO to pin numbering
detection_count = 0

readyContour = Queue.Queue()
ggframes = Queue.Queue()

scheme = 0




def applyGaussian(frame,ggframes):
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  if useGaussian:
    gray = cv2.GaussianBlur(gray, (gaussianPixels, gaussianPixels), 0)
  ggframes.put(gray)


def calculateFrameDiff(firstFrame,gray,readyContour):
  frameDelta = cv2.absdiff(firstFrame, gray)
  thresh = cv2.threshold(frameDelta, thresholdLimit, 255, cv2.THRESH_BINARY)[1]
  thresh = cv2.dilate(thresh, None, iterations=dilationPixels) # dilate thresh
  _, contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #find contours
  readyContour.put(contours)


def main():

    setup_pi()

    try:
        while True:
            time.sleep(2)
            if GPIO.input(pir_pin) == True:  # If PIR pin goes high, motion is detected
                detection_count = detection_count + 1
                print("Motion Detected! count:{}".format(detection_count))
                GPIO.output(led_pin, True)  # Turn on LED
                time.sleep(0.5)  # Keep LED on for 4 seconds
                GPIO.output(led_pin, False)  # Turn off LED

                if scheme is 0:
                    camera_confirmation = use_camera()


                time.sleep(0.1)

    except KeyboardInterrupt:  # Ctrl+c
        pass  # Do nothing, continue to finally

    finally:
        GPIO.output(led_pin, False)  # Turn off LED in case left on
        GPIO.cleanup()  # reset all GPIO
        print("Program ended")





if  __name__ =='__main__':
    main()