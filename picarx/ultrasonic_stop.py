#Robot Imports:
import numpy as np
import os
import picarx_improved
import logging
import time
from threading import Event

try:
    #See if we can load the robot hat drivers (not present off car)
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils

    on_the_robot = True

except ImportError:
    #Otherwise Load our 'Sim' copies
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils

    on_the_robot = False

class sensor():
    
    SOUND_SPEED = 343.3 # ms

    def __init__(self, trigNum="D2", echoNum="D3", timeout=0.02):
        
        self.timeout = timeout

        self.trig = Pin(trigNum)
        self.echo = Pin(echoNum, mode=Pin.IN, pull=Pin.PULL_DOWN)

    def _read(self):
        if on_the_robot:
            self.trig.off()
            time.sleep(0.001)
            self.trig.on()
            time.sleep(0.00001)
            self.trig.off()

            pulse_end = 0
            pulse_start = 0
            timeout_start = time.time()

            while self.echo.value() == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > self.timeout:
                    return -1
            while self.echo.value() == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > self.timeout:
                    return -1
            if pulse_start == 0 or pulse_end == 0:
                return -2

            during = pulse_end - pulse_start
            cm = round(during * self.SOUND_SPEED / 2 * 100, 2)
            return cm
        else:
            return np.random.uniform() * 10

    def read(self, times=10):
        for i in range(times):
            a = self._read()
            if a != -1:
                return a
        return -1

    def close(self):
        self.trig.close()
        self.echo.close()

class interpreter():
    def __init__(self,thresh):
        self.thresh = thresh

    def process(self,reading):
        if reading == -1:
            return False
        else:    
            return (reading > self.thresh)
        

if __name__ == "__main__":
    
    s = sensor()
    
    thresh = 5

    i = interpreter(thresh)

    while True:

        reading = s.read()

        result = i.process(reading)

        print(f"Got: {reading}, Move: {result}")

