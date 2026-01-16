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

# Extend system path to include script directory
sys.path.append(os.path.join(os.getcwd(), 'scripts'))

from hiwonder import HiwonderRobot
import utils


def joystick_control(robot, dt, new_thetalist):
    # get the latest gamepad command
    cmd = robot.gamepad.cmdlist[-1]

    max_rate = 400  # 400 x 0.1 = 40 deg/s
    
    new_thetalist[0] += dt * max_rate * cmd.arm_j1
    new_thetalist[1] += dt * max_rate * cmd.arm_j2
    new_thetalist[2] += dt * max_rate * cmd.arm_j3
    new_thetalist[3] += dt * max_rate * cmd.arm_j4
    new_thetalist[4] += dt * max_rate * cmd.arm_j5
    new_thetalist[5] += dt * max_rate * cmd.arm_ee

    new_thetalist = robot.enforce_joint_limits(new_thetalist)
    new_thetalist = [round(theta,3) for theta in new_thetalist]
    print(f'[DEBUG] Current thetalist (deg) = {robot.get_joint_values()}') 
    print(f'[DEBUG] Commanded thetalist (deg) = {new_thetalist}')       
    
    # set new joint angles
    robot.set_joint_values(new_thetalist, duration=dt, radians=False)



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        
        control_hz = 20 
        dt = 1 / control_hz

        thetalist = robot.get_joint_values().copy()

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                joystick_control(robot, dt, thetalist)


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


