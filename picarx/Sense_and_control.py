import os
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
import atexit
import math
import picarx_improved

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")

logging.getLogger().setLevel(logging.DEBUG)

on_the_robot = True
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = True
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..")))
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = False
import time

class Sensor():
    def __init__(self, pin_names):
        self.pins = []
        self.pin_val = []
        for pin in pin_names:
            self.pins.append(ADC(pin))
            self.pin_val.append(0.0)
        
        
    def read(self):
        self.pin_val = []
        for i, pin in enumerate(self.pins):
            self.pin_val.append(pin.read_voltage())
        # logging.debug(f"Pin values {self.pin_val}")
        return self.pin_val
    
class Interpreter():
    def __init__(self, line_threshold=1.65, is_dark=1):
        self.line_threshold = line_threshold
        if is_dark:
            self.sign = 1
        else:
            self.sign = -1
        # self.state = []
        self.last_val = 0

    def process(self, pin_vals):
        # if self.state == []:
        #     self.state = pin_vals
        left_diff = pin_vals[1] - pin_vals[0]
        right_diff = pin_vals[1] - pin_vals[2]
        max_val = max(abs(val) for val in pin_vals)
        #check threshold
        match(abs(left_diff) > self.line_threshold, abs(right_diff) > self.line_threshold):
            case(False, False):
                #Line is either centered or non existant
                if abs(self.last_val) > self.line_threshold:
                    return self.last_val
                else:
                    self.last_val = 0
                    return 0
            case(True, False):
                #pin is under left
                if self.sign * left_diff > 0:
                    self.last_val = abs(left_diff/max_val)
                    return abs(left_diff/max_val)
                else:
                    self.last_val = -1 * abs(right_diff/max_val)
                    return -1 * abs(right_diff/max_val)
            case(False, True):
                if self.sign * right_diff > 0:
                    self.last_val = -1 * abs(right_diff/max_val)
                    return -1 * abs(right_diff/max_val)
                else:
                    self.last_val = abs(left_diff/max_val)
                    return abs(left_diff/max_val)
            case(True, True):
                #line is centered
                #could interpolate to adjust
                self.last_val = 0
                return 0
            
    
class Control():
    def __init__(self, px, scaling_factor = 1):
        self.scaling_factor = scaling_factor
        self.px = px
    
    def update_steer(self, position):
        #find angle

        self.px.set_dir_servo_angle(-1 * self.scaling_factor * position * self.px.DIR_MAX)
        logging.debug(f"Steering angle: {self.scaling_factor * position * self.px.DIR_MAX}")

if __name__ == "__main__":
    pin_names = ["A0", "A1", "A2"]
    px = picarx_improved.Picarx()
    sensor = Sensor(pin_names=pin_names)
    interpreter = Interpreter(line_threshold=0.2, is_dark=1)
    control = Control(px, 1.75)

    line = 0
    pin_vals = []
    while True:
        pin_vals = sensor.read()
        logging.debug(f"Pin Vals: {pin_vals}")
        line = interpreter.process(pin_vals=pin_vals)
        logging.debug(f"Line Position {line}")
        control.update_steer(position=line)
        time.sleep(0.2)
        px.forward(20)


