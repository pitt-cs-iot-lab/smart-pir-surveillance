from picamera.array import PiRGBArray
from threading import Thread
from picamera import PiCamera
import RPi.GPIO as GPIO
import Queue
import time
import cv2

debug = 1

class CameraSurveillance:

    def __init__(self):
        self.debug = 1

        self.ready_contour = Queue.Queue()
        self.ggframes = Queue.Queue()

        # Video or camera
        self.camera = PiCamera()
        self.camera.vflip = True
        self.camera.resolution = (1024, 864)
        self.camera.framerate = 32
        self.raw_capture = PiRGBArray(self.camera, size=(1024, 864))

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
        self.width_ratio = 0.40
        self.min_area = 40 * 40
        self.threshold_limit = 20
        self.dilation_pixels = 20  # 10
        self.use_gaussian = 1
        self.gaussian_pixels = 31
        self.contours = []
        self.up_date_frame = 0
        self.first_time = True
        self.fall_state = ""

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

    def start_surveillance(self):

        try:
            # loop for each frame in video
            for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):

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

                    if w > h * self.width_ratio:
                        #GPIO.output(20, True)
                        self.fall_state = "Alarm!"
                        self.box_color = self.red_box
                        print "Alarm: " + format(time.time())
                    else:
                        self.fall_state = ""
                        self.box_color = self.green_box

                    self.lastW[self.i] = w
                    self.lastH[self.i] = h
                    # cv2.putText(frame,"{}".format(cv2.contourArea(contour)), (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                    cv2.putText(frame, "{}".format(self.i), (x, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.box_color, 1)
                    cv2.putText(frame, "{}".format(self.fall_state), (x + 22, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.box_color, 1)
                    detect_status = "Ok"
                    self.i += 1

                # Hud + fps
                if debug:
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
            cv2.destroyAllWindows()
            pass
