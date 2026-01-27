import os
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
import atexit
import math
import picarx_improved
import cv2 as cv
import numpy as np

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")

logging.getLogger().setLevel(logging.DEBUG)

on_the_robot = True
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
    from picamera2 import Picamera2
    on_the_robot = True
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..")))
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = False
import time


class ImageProcessing():
    def __init__(self, is_dark=True, on_the_robot = True, crop = [640, 480]):
        self.is_dark = is_dark
        if on_the_robot:
            self.picam = Picamera2()
            config = self.picam.create_preview_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.picam.configure(config)
            self.picam.start()
        else:
            self.cam = cv.VideoCapture(-1)
            # self.cam = cv.VideoCapture("/dev/v4l/by-id/usb-Azurewave_Integrated_Camera_SN0001-video-index0")
        time.sleep(0.5)
        self.crop = crop
        return
    def get_binary(self, verbose=False):
        if on_the_robot:
            frame = self.picam.capture_array()
            frame_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        else:
            ret, frame_bgr = self.cam.read()
            print("Frame read:", ret)
            if not ret or frame_bgr is None:
                print("Camera failed, returning default")
                return None

        frame_grey = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
        frame_grey = cv.GaussianBlur(frame_grey, (5,5), 0)
        if self.is_dark:
            binary_thresh = cv.THRESH_BINARY_INV
        else:
            binary_thresh = cv.THRESH_BINARY
        #frame_binary = cv.adaptiveThreshold(frame_grey, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, binary_thresh, 11, 2)
        ret, frame_binary = cv.threshold(frame_grey, 0, 255, binary_thresh + cv.THRESH_OTSU)
        #crop the frame
        h, w = frame_binary.shape[:2]
        w_start = max(0, w//2-self.crop[0]//2)
        w_end = min(w, w//2 + self.crop[0]//2)
        crop_frame_binary = frame_binary[h-self.crop[1]:h, w_start:w_end]
        if verbose:
           # cv.imshow("Capture", frame_bgr)
           # cv.waitKey(100)
            # cv.imshow("Binary", frame_binary)
            # cv.waitKey(100)
            cv.imshow("Cropped Binary", crop_frame_binary)
            cv.waitKey(100)
        
        return crop_frame_binary
    
    def get_contour(self, frame):
        cnts, hierarchy = cv.findContours(frame, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
        contour = None
        if cnts is not None and len(cnts) > 0:
            contour = max(cnts, key = cv.contourArea)
        if contour is None:
            return None, None, None
        rect = cv.minAreaRect(contour)
        box = cv.boxPoints(rect)
        box = box.astype(int)
        box = self.order_box(box)
        return frame, contour, box
    
    def process_image(self, draw = False):
        frame = self.get_binary(verbose=draw)
        if frame is None:
            return 90, 0
        h, w = frame.shape[:2]
        frame, contour, box = self.get_contour(frame)
        p1, p2 = self.calc_box_vector(box)
        angle = self.get_vert_angle(p1, p2, w, h)
        shift = self.get_horz_shift(p1[0], w)

        if draw:
            cv.drawContours(frame, [contour], -1, (0,0,255), 3)
            cv.drawContours(frame,[box],0,(255,0,0),2)
            cv.line(frame, p1, p2, (0, 255, 0), 3)
            msg_a = "Angle {0}".format(int(angle))
            msg_s = f"Shift {shift:.3f}"

            cv.putText(frame, msg_a, (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv.putText(frame, msg_s, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv.imshow("Image", frame)
            cv.waitKey(100)

        return angle, shift
    
    def order_box(self, box):
        srt = np.argsort(box[:, 1])
        btm1 = box[srt[0]]
        btm2 = box[srt[1]]

        top1 = box[srt[2]]
        top2 = box[srt[3]]

        bc = btm1[0] < btm2[0]
        btm_l = btm1 if bc else btm2
        btm_r = btm2 if bc else btm1

        tc = top1[0] < top2[0]
        top_l = top1 if tc else top2
        top_r = top2 if tc else top1

        return np.array([top_l, top_r, btm_r, btm_l])
    
    # def shift_box(self, box, w, h):
    #     return np.array([[box[0][0] + w,box[0][1] + h],[box[1][0] + w,box[1][1] + h], [box[2][0] + w,box[2][1] + h],[box[3][0] + w,box[3][1] + h]])

    def calc_box_vector(self, box):
        v_side = self.calc_line_length(box[0], box[3])
        h_side = self.calc_line_length(box[0], box[1])
        idx = [0, 1, 2, 3]
        if v_side < h_side:
            idx = [0, 3, 1, 2]
        return ((box[idx[0]][0] + box[idx[1]][0]) // 2, (box[idx[0]][1] + box[idx[1]][1]) // 2), ((box[idx[2]][0] + box[idx[3]][0]) // 2, (box[idx[2]][1]  +box[idx[3]][1]) // 2)
    
    def calc_line_length(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def get_horz_shift(self, x, w):
        hw = w / 2
        return min(max((hw - x) / hw, -1),1)

    def calc_rect_area(self, rect_points):
        a = self.calc_line_length(rect_points[0], rect_points[1])
        b = self.calc_line_length(rect_points[1], rect_points[2])
        return a * b

    def get_vert_angle(self, p1, p2, w, h):
        px1 = p1[0] - w/2
        px2 = p2[0] - w/2
        
        py1 = h - p1[1]
        py2 = h - p2[1]

        angle = 90
        if px1 != px2:
            a, b = self.calc_line(px1, py1, px2, py2)
            angle = 0
            if a != 0:
                x0 = -b/a
                y1 = 1.0
                x1 = (y1 - b) / a
                dx = x1 - x0
                tg = y1 * y1 / dx / dx
                angle = 180 * np.arctan(tg) / np.pi
                if a < 0:
                    angle = 180 - angle
        return angle
    
    def calc_line(self, x1, y1, x2, y2):
        a = float(y2 - y1) / (x2 - x1) if x2 != x1 else 0
        b = y1 - a * x1
        return a, b
    
class ControlForImage():
    def __init__(self, px, scaling_factor = 1):
        self.px = px
        self.scaling_factor = scaling_factor
        self.px.set_cam_tilt_angle(-35)

    def update_steer(self, angle, shift):
        shift_angle = -1 * self.scaling_factor * shift * self.px.DIR_MAX
        line_angle = -1 * (90-angle)
        total_turn_angle = shift_angle + line_angle
        #check if greater than max, set to inverse max, and reverse
        if abs(total_turn_angle) > self.px.DIR_MAX:
            self.px.set_dir_servo_angle(-1 *total_turn_angle)
            return -1
        #else set steering angle
        else:
            self.px.set_dir_servo_angle(total_turn_angle)
            return 1

if __name__ == "__main__":
    px = picarx_improved.Picarx()
    image_processor = ImageProcessing(is_dark=True, on_the_robot=on_the_robot, crop=[640, 160])
    control = ControlForImage(px, scaling_factor=1.75)
    angle = 90
    shift = 0
    while True:
        angle, shift = image_processor.process_image(draw=True)
        logging.debug(f"Line Angle: {angle} | Line Shift: {shift}")
        direction = control.update_steer(angle, shift)
        time.sleep(0.1)
        px.forward(direction * 20)
