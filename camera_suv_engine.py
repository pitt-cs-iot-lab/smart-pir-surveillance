from picamera.array import PiRGBArray
from threading import Thread
from picamera import PiCamera
import RPi.GPIO as GPIO
import Queue
import time
import cv2


GPIO.setmode(GPIO.BOARD) #Set GPIO to pin numbering
led_buzzer_pin = 10 #Assign pin 10 to LED
GPIO.setup(led_buzzer_pin, GPIO.OUT) #Setup GPIO pin for LED as output

class CameraSurveillance:
    def __init__(self, e_debug=True, alarm_threshold=1):
        self.debug = e_debug

        self.alarm_threshold = alarm_threshold
        self.alarm_counter = 0
        self.intruder_detected = False

        self.ready_contour = Queue.Queue()
        self.ggframes = Queue.Queue()

        # Video or camera
        self.camera = PiCamera()
        self.camera.vflip = True
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.raw_capture = PiRGBArray(self.camera, size=(640, 480))

        # Some time to initialize camera
        time.sleep(1.0)

        self.first_frame = None
        self.gray = None
        self.start = time.time()
        self.i = 0
        self.lastH = [0] * 100
        self.lastW = [0] * 100
        self.box_position = [0] * 100

        # Detection parameters
        self.alarm_ratio = 1.40
        self.min_area = 40 * 40
        self.threshold_limit = 20
        self.dilation_pixels = 20  # 10
        self.use_gaussian = 1
        self.gaussian_pixels = 31
        self.contours = []
        self.up_date_frame = 0
        self.first_time = True
        self.alarm_text = ""

        self.red_box = (0, 0, 255)
        self.green_box = (124, 252, 0)
        self.box_color = self.red_box

    def _convert_frame(self, frame):
        r = 750.0 / frame.shape[1]
        dim = (750, int(frame.shape[0] * r))
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        return frame

    def _apply_gaussian(self, frame, ggframes):
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_gaussian:
            self.gray = cv2.GaussianBlur(self.gray, (self.gaussian_pixels, self.gaussian_pixels), 0)
        ggframes.put(self.gray)

    def _calculate_frame_diff(self, first_frame, gray, ready_contour):
        frame_delta = cv2.absdiff(first_frame, gray)
        thresh = cv2.threshold(frame_delta, self.threshold_limit, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=self.dilation_pixels)  # dilate thresh
        _, self.contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # find contours
        ready_contour.put(self.contours)

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.camera.close()
        except Exception as ex:
            pass

    def __enter__(self):
        return self

    def start_surveillance(self, surveillance_duration=50):
        processed_frame = 0
        try:
            # loop for each frame in video
            for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):

                processed_frame += 1

                detect_status = "Empty"
                frame = frame.array
                frame = self._convert_frame(frame)

                if self.first_time is True:
                    self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    if self.use_gaussian:
                        self.gray = cv2.GaussianBlur(self.gray, (self.gaussian_pixels, self.gaussian_pixels), 0)
                    self.first_time = False

                gthread = Thread(target=self._apply_gaussian, args=(frame, self.ggframes))
                if gthread.isAlive() is False:
                    gthread.start()

                if self.ggframes.empty() is False:
                    self.gray = self.ggframes.get_nowait()

                if self.first_frame is None:
                    self.raw_capture.truncate(0)
                    time.sleep(1.0)  # let camera autofocus + autosaturation settle
                    self.first_frame = self.gray
                    continue

                thread = Thread(target=self._calculate_frame_diff, args=(self.first_frame, self.gray, self.ready_contour))
                if thread.isAlive() is False:
                    thread.start()

                if self.ready_contour.empty() is False:
                    self.contours = self.ready_contour.get_nowait()

                #if not self.contours:
                #    GPIO.output(20, False)

                for contour in self.contours:
                    if cv2.contourArea(contour) < self.min_area:
                        continue

                    # Drawing rect over contour
                    (x, y, w, h) = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), self.box_color, 2)
                    self.box_position[self.i] = x + y

                    if h > w * self.alarm_ratio:
                        #GPIO.output(20, True)
                        self.alarm_text = "Alarm!"
                        self.box_color = self.red_box
                        self.alarm_counter += 1
                        print("Alarm Counter: {}".format(self.alarm_counter))
                        
                        GPIO.output(led_buzzer_pin, True) #Turn on LED
                        time.sleep(0.1) #Keep LED on for 4 seconds
                        GPIO.output(led_buzzer_pin, False) #Turn on LED
                        
                        if self.alarm_counter >= self.alarm_threshold:
                            self.intruder_detected = True
                            print("Alarm: " + format(time.time()))

                    else:
                        self.alarm_text = ""
                        self.box_color = self.green_box

                    self.lastW[self.i] = w
                    self.lastH[self.i] = h
                    # cv2.putText(frame,"{}".format(cv2.contourArea(contour)), (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                    cv2.putText(frame, "{}".format(self.i), (x, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.box_color, 1)
                    cv2.putText(frame, "{}".format(self.alarm_text), (x + 22, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.box_color, 1)
                    detect_status = "Ok"
                    self.i += 1

                if self.intruder_detected is True:
                    time.sleep(5.0)
                    self.camera.close()
                    return True

                if processed_frame > surveillance_duration:
                    self.camera.close()
                    return False

                # Hud + fps
                if self.debug is True:
                    end = time.time()
                    seconds = end - self.start
                    fps = round((1 / seconds), 1)
                    self.start = time.time()

                    cv2.putText(frame, "Detect: {}".format(detect_status), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 140, 255), 1)
                    cv2.putText(frame, "FPS: {}".format(fps), (400, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 140, 255), 1)
                    # cv2.imshow("frameDelta", frameDelta)
                    # cv2.imshow("Thresh", thresh)
                    # cv2.imshow("first_frame", first_frame)

                cv2.imshow("Feed", frame)

                self.i = 0

                self.raw_capture.truncate(0)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("n"):
                    self.first_frame = None

        except KeyboardInterrupt:
            self.camera.release()
            GPIO.cleanup() #reset all GPIO
            cv2.destroyAllWindows()
            pass
