from camera_suv_engine import *
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)  # Set GPIO to pin numbering
pir_pin = 8  # Assign pin 8 to PIR
led_pin = 10  # Assign pin 10 to LED
GPIO.setup(pir_pin, GPIO.IN)  # Setup GPIO pin PIR as input
GPIO.setup(led_pin, GPIO.OUT)  # Setup GPIO pin for LED as output
print("Sensor initializing . . .")
time.sleep(2)  # Give sensor time to startup
print("Active")
print("Press Ctrl+c to end program")

scheme = 0


def main():

    try:
        while True:
            time.sleep(2)
            if GPIO.input(pir_pin) is True:  # If PIR pin goes high, motion is detected
                detection_count = detection_count + 1
                print("Motion Detected! count:{}".format(detection_count))
                GPIO.output(led_pin, True)  # Turn on LED
                time.sleep(0.5)  # Keep LED on for 4 seconds
                GPIO.output(led_pin, False)  # Turn off LED

                if scheme is 0:
                    suvCam = CameraSurveillance()
                    suvCam.start_surveillance()

                time.sleep(0.1)

    except KeyboardInterrupt:  # Ctrl+c
        pass  # Do nothing, continue to finally

    finally:
        GPIO.output(led_pin, False)  # Turn off LED in case left on
        GPIO.cleanup()  # reset all GPIO
        print("Program ended")


if __name__ == '__main__':
    main()
