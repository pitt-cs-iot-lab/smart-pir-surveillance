from camera_suv_engine import *


def main():

    suvCam = CameraSurveillance(True, 30)
    intruder_detected = suvCam.start_surveillance()

    if intruder_detected is True:
        print "A intruder was detected!!!!!"
    else:
        print "No Intruder was detected. PIR false alarm"


if __name__ == '__main__':
    main()
