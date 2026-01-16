# main.py
"""
Main Application Script
----------------------------
Coordinates gamepad input and robot control.
"""

import sys, os
import time
import traceback
import numpy as np
import cv2

# Extend system path to include script directory
sys.path.append(os.path.join(os.getcwd(), 'scripts'))

from hiwonder import HiwonderRobot
from arm_models import FiveDOFRobot
import utils as ut


# Initialize components
robot = HiwonderRobot()
model = FiveDOFRobot()


class CameraExampleFSM():
    def __init__(self, dt=0.05):
        self.t = 0.0
        self.dt = dt
        self.camera = cv2.VideoCapture(0)
        self.new_thetalist = robot.get_joint_values().copy()

    def js_control(self):
        cmd = robot.gamepad.cmdlist[-1]

        max_rate = 400  # 400 x 0.1 = 40 deg/s
        self.new_thetalist[0] += self.dt * max_rate * cmd.arm_j1
        self.new_thetalist[1] += self.dt * max_rate * cmd.arm_j2
        self.new_thetalist[2] += self.dt * max_rate * cmd.arm_j3
        self.new_thetalist[3] += self.dt * max_rate * cmd.arm_j4
        self.new_thetalist[4] += self.dt * max_rate * cmd.arm_j5
        self.new_thetalist[5] += self.dt * max_rate * cmd.arm_ee

        self.new_thetalist = robot.enforce_joint_limits(self.new_thetalist)
        self.new_thetalist = [round(theta,3) for theta in self.new_thetalist]   
        
        # set new joint angles
        robot.set_joint_values(self.new_thetalist, duration=self.dt, radians=False)


    def process_image(self):
        ret, frame = self.camera.read()
        self.t = 0.0 # reset timer

        if ret:
            print("Saving to frame.png")
            cv2.imwrite("frame.png", frame)
        else:
            print("Failed to capture frame")


    def step(self):
        # process image
        if self.t >= 0.5:
            self.process_image()

        # control robot
        self.js_control()

        self.t += self.dt
        print(f'Time: [{self.t}]')



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:
       
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()

        fsm = CameraExampleFSM(dt=dt)

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                fsm.step()


            elapsed = time.time() - t_start
            remaining_time = dt - elapsed
            if remaining_time > 0:
                time.sleep(remaining_time)

            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        robot.shutdown_robot()




if __name__ == "__main__":
    main()


