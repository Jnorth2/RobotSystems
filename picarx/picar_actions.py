import atexit
import math
import picarx_improved
import readchar
import time


def bad_driving(px, speed, angle, dist):
    """
    Function to drive at a set speed, angle, and distance.

    Parameters:
    -----------
    px: Picarx object
        The Picarx object to control the car.
    speed: float
        The speed to drive at [-100:-100].
    angle: float
        The angle to turn the car (in degrees).
    dist: float
        The distance to drive (in PiCar Units).
    """
    t = dist/speed
    px.set_dir_servo_angle(angle)
    time.sleep(0.1)
    px.forward(speed)
    time.sleep(t)
    px.stop()

def crashing_parallel_park(px, direction):
    """
    Function to parallel park with open loop control.
    Parameters:
    -----------
    px: Picarx object
        The Picarx object to control the car.
    """
    #Step 1: Turn wheels and Reverse a distance
    bad_driving(px, -50, direction*30, 0.5)
    #Step 2: Straighten wheels and Reverse a distance
    bad_driving(px, -50, 0, 0.5)
    #Step 3: Turn wheels in opposite direction and Reverse a distance
    bad_driving(px, -50, -direction*30, 0.5)
    #Step 4: Straighten wheels and Reverse a distance
    bad_driving(px, -50, 0, 0.5)
    #Step 5: Drive Forward a distance
    bad_driving(px, 50, 0, 0.2)

def idiot_three_point_turn(px, direction):
    """
    Function to perform a three point turn with open loop control.
    Parameters:
    -----------
    px: Picarx object
        The Picarx object to control the car.
    direction: {1, -1}
        The direction to turn.
    """
    #Step 1: Turn wheels and Drive Forward a distance
    bad_driving(px, 50, direction*30, 0.5)
    #Step 2: Straighten wheels and Reverse a distance
    bad_driving(px, -50, direction*-30, 0.5)
    #Step 3: Turn wheels in opposite direction and Drive Forward a distance
    bad_driving(px, 50, 0, 0.5)



if __name__ == "__main__":
    px = picarx_improved.Picarx()
    atexit.register(px.stop)
    angle = 0
    while True:
        command = input("Enter command (w, a, s, d, pl, pr, 3l, 3r, x(stop)): ")
        if command == "w":
            bad_driving(px, 50, angle, 0.5)
        elif command == "s":
            bad_driving(px, -50, angle, 0.5)
        elif command == "a":
            angle += 15
            bad_driving(px, 0, angle, 0.5)
        elif command == "d":
            angle -= 15
            bad_driving(px, 0, angle, 0.5)
        elif command == "pl":
            crashing_parallel_park(px, 1)
        elif command == "pr":
            crashing_parallel_park(px, -1)
        elif command == "3l":
            idiot_three_point_turn(px, 1)
        elif command == "3r":
            idiot_three_point_turn(px, -1)
        elif command == "x":
            px.stop()
        else:
            print("Invalid command.")



