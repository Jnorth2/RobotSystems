import os
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
import atexit
import math
import picarx_improved
import sys
sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..")))
from RossROS import rossros

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
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = False
import time

class Sensor():
    def __init__(self, pin_names, ultrasonic_pin_names):
        self.pins = []
        self.pin_val = []
        for pin in pin_names:
            self.pins.append(ADC(pin))
            self.pin_val.append(0.0)
        
        #setup ultrasonic sensor
        self.ultrasonic = Ultrasonic(Pin(ultrasonic_pin_names[0]), Pin(ultrasonic_pin_names[1], mode=Pin.IN, pull=Pin.PULL_DOWN))
        self.ultrasonic_val = self.ultrasonic.read()

    def read(self):
        self.pin_val = []
        for i, pin in enumerate(self.pins):
            self.pin_val.append(pin.read_voltage())
        # logging.debug(f"Pin values {self.pin_val}")
        return self.pin_val
    
    def read_ultrasonic(self):
        self.ultrasonic_val = self.ultrasonic.read()
        return self.ultrasonic_val

class Interpreter():
    def __init__(self, line_threshold=1.65, is_dark=1, stop_thres=2):
        self.line_threshold = line_threshold
        self.stop_thres = stop_thres
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
                if self.last_val > 0.2:
                    self.last_val = 1 
                    return self.last_val
                elif self.last_val < -0.2:
                    self.last_val = -1
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
            
    def process_ultrasonic(self, ultrasonic_val):
        if ultrasonic_val < self.stop_thres and ultrasonic_val >= 0:
            return True
        return False

                
class Control():
    def __init__(self, px, scaling_factor = 1):
        self.scaling_factor = scaling_factor
        self.px = px
    
    def update_steer(self, position, stop=False):
        #find angle
        if stop:
            self.px.forward(0)
        elif abs(position) == 1:
            self.px.set_dir_servo_angle(position * self.px.DIR_MAX)
            logging.debug(f"Steering angle: {self.px.DIR_MAX * position} and Direction = -1")
            px.forward(-20)
        else:
            self.px.set_dir_servo_angle(-1 * self.scaling_factor * position * self.px.DIR_MAX)
            logging.debug(f"Steering angle: {-1 * self.scaling_factor * position * self.px.DIR_MAX}")
            px.forward(20)

if __name__ == "__main__":
    pin_names = ["A0", "A1", "A2"]
    ultrasonic_pins = ["D2", "D3"]
    px = picarx_improved.Picarx()
    sensor = Sensor(pin_names=pin_names, ultrasonic_pin_names=ultrasonic_pins)
    interpreter = Interpreter(line_threshold=0.3, is_dark=1, stop_thres=2)
    control = Control(px, 1.75)

    term_bus = rossros.Bus(False, "Timer Termination Bus")
    timer = rossros.Timer(output_buses=term_bus,duration=10, delay=0.03,termination_buses= term_bus, name="Termination Timer")
    sensor_bus = rossros.Bus([0, 0, 0], name="Sensor Bus")
    position_bus = rossros.Bus(0, name="Position Bus")
    us_sensor_bus = rossros.Bus(0, name="Ultrasonic Sensor Bus")
    move_bus = rossros.Bus(0, name="Distance Bus")

    ross_sensor = rossros.Producer(sensor.read, sensor_bus, delay= 0.1, termination_buses=term_bus, name="Sensor")
    ross_us_sensor = rossros.Producer(sensor.read_ultrasonic, us_sensor_bus, delay=0.1, termination_buses=term_bus, name="Ultrasonic Sensor")
    ross_move = rossros.ConsumerProducer(interpreter.process_ultrasonic, ross_us_sensor, move_bus, delay=0.1, termination_buses=term_bus, name="Move Process")
    ross_process = rossros.ConsumerProducer(interpreter.process, sensor_bus, position_bus, delay=0.1, termination_buses=term_bus, name="Process")
    ross_control = rossros.Consumer(control.update_steer, [position_bus, move_bus], delay=0.1, termination_buses=term_bus, name="Control")

    list_of_processes = [
        timer,
        ross_sensor,
        ross_us_sensor,
        ross_move,
        ross_process,
        ross_control,
    ]

    rossros.runConcurrently(list_of_processes)


