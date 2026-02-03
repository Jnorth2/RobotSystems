import os
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
import atexit
import math
import picarx_improved
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from readerwriterlock import rwlock

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")

logging.getLogger().setLevel(logging.DEBUG)

shutdown_event = Event()

def handle_exception(future):
    exception = future.exception()
    if exception:
        print(f"Exception in Worker thread: {exception}")

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

class Bus():
    def __init__(self, data = None):
        self.data = data
        self.lock = rwlock.RWLockWriteD()


    def write(self, data):
        with self.lock.gen_wlock():
            self.data = data
    
    def read(self):
        with self.lock.gen_rlock():
            return self.data

class Sensor():
    def __init__(self, pin_names):
        self.pins = []
        self.pin_val = []
        for pin in pin_names:
            self.pins.append(ADC(pin))
            self.pin_val.append(0.0)
        
        
    def read(self, bus : Bus, delay_time):
        while not shutdown_event.is_set():
            self.pin_val = []
            for i, pin in enumerate(self.pins):
                self.pin_val.append(pin.read_voltage())
            # logging.debug(f"Pin values {self.pin_val}")
            bus.write(self.pin_val)
            time.sleep(delay_time)

class Interpreter():
    def __init__(self, line_threshold=1.65, is_dark=1):
        self.line_threshold = line_threshold
        if is_dark:
            self.sign = 1
        else:
            self.sign = -1
        # self.state = []
        self.last_val = 0

    def process(self, bus_in, bus_out, delay_time):
        # if self.state == []:
        #     self.state = pin_vals
        while not shutdown_event.is_set():
            pin_vals = bus_in.read()
            if not pin_vals:
                time.sleep(delay_time)
                continue
            left_diff = pin_vals[1] - pin_vals[0]
            right_diff = pin_vals[1] - pin_vals[2]
            max_val = max(abs(val) for val in pin_vals)
            #check threshold
            match(abs(left_diff) > self.line_threshold, abs(right_diff) > self.line_threshold):
                case(False, False):
                    #Line is either centered or non existant
                    if self.last_val > 0.2:
                        self.last_val = 1 
                        bus_out.write(self.last_val)
                    elif self.last_val < -0.2:
                        self.last_val = -1
                        bus_out.write(self.last_val)
                    else:
                        self.last_val = 0
                        bus_out.write(0)
                case(True, False):
                    #pin is under left
                    if self.sign * left_diff > 0:
                        self.last_val = abs(left_diff/max_val)
                        bus_out.write( abs(left_diff/max_val))
                    else:
                        self.last_val = -1 * abs(right_diff/max_val)
                        bus_out.write( -1 * abs(right_diff/max_val))
                case(False, True):
                    if self.sign * right_diff > 0:
                        self.last_val = -1 * abs(right_diff/max_val)
                        bus_out.write( -1 * abs(right_diff/max_val))
                    else:
                        self.last_val = abs(left_diff/max_val)
                        bus_out.write(abs(left_diff/max_val))
                case(True, True):
                    #line is centered
                    #could interpolate to adjust
                    self.last_val = 0
                    bus_out.write(0)
            time.sleep(delay_time)
                
    
class Control():
    def __init__(self, px, scaling_factor = 1):
        self.scaling_factor = scaling_factor
        self.px = px
    
    def update_steer(self, bus, delay_time):
        #find angle

        while not shutdown_event.is_set():
            position = bus.read()
            if not position:
                time.sleep(delay_time)
                continue
            if abs(position) == 1:
                self.px.set_dir_servo_angle(position * self.px.DIR_MAX)
                logging.debug(f"Steering angle: {self.px.DIR_MAX * position} and Direction = -1")
                px.forward(-20)
            else:
                self.px.set_dir_servo_angle(-1 * self.scaling_factor * position * self.px.DIR_MAX)
                logging.debug(f"Steering angle: {-1 * self.scaling_factor * position * self.px.DIR_MAX}")
                px.forward(20)
            time.sleep(delay_time)

if __name__ == "__main__":
    pin_names = ["A0", "A1", "A2"]
    px = picarx_improved.Picarx()
    sensor = Sensor(pin_names=pin_names)
    interpreter = Interpreter(line_threshold=0.3, is_dark=1)
    control = Control(px, 1.75)

    sensor_bus = Bus()
    process_bus = Bus()

    sensor_delay = 0.1
    process_delay = 0.1
    control_delay = 0.1

    futures = []
    with ThreadPoolExecutor(max_workers=3) as executor:
        esensor = executor.submit(sensor.read, sensor_bus,  sensor_delay)
        esensor.add_done_callback(handle_exception)
        futures.append(esensor)
        einterpretor = executor.submit(interpreter.process, sensor_bus, process_bus, process_delay)
        einterpretor.add_done_callback(handle_exception)
        futures.append(einterpretor)
        econtrol = executor.submit(control.update_steer, process_bus, control_delay)
        econtrol.add_done_callback(handle_exception)
        futures.append(econtrol)

        try:
            while not shutdown_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Shutting down")
            shutdown_event.set()
        finally:
            executor.shutdown()

    # while True:
    #     pin_vals = sensor.read()
    #     logging.debug(f"Pin Vals: {pin_vals}")
    #     line = interpreter.process(pin_vals=pin_vals)
    #     logging.debug(f"Line Position {line}")
    #     direction = control.update_steer(position=line)
    #     time.sleep(0.1)


