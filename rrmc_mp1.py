# main.py
"""
Main Application Script
----------------------------
Coordinates gamepad input and robot control.
"""

import sys, os
import time
import threading
import traceback
import numpy as np

# Extend system path to include script directory
sys.path.append(os.path.join(os.getcwd(), 'scripts'))

from hiwonder import HiwonderRobot
from arm_models import FiveDOFRobot
import utils



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        model = FiveDOFRobot()
        
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()
        
        new_thetalist = robot.get_joint_values()

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                cmd = robot.gamepad.cmdlist[-1]

                if cmd.arm_home:
                    robot.move_to_home_position()

                curr_thetalist = robot.get_joint_values()

                print(f'[DEBUG] Current thetalist (deg) = {robot.get_joint_values()}') 

                vel = [cmd.arm_vx, cmd.arm_vy, cmd.arm_vz]
                thetalist_dot = model.calc_velocity_kinematics(curr_thetalist, vel, radians=False)

                print(f'[DEBUG] linear vel: {[round(vel[0], 3), round(vel[1], 3), round(vel[2], 3)]}')
                print(f'[DEBUG] thetadot (deg/s) = {[round(td,2) for td in thetalist_dot]}')
                
                # linear velocity control
                K = 2 # small gain value 
                for i in range(5):
                    new_thetalist[i] += dt * K * thetalist_dot[i]

                new_thetalist = robot.enforce_joint_limits(new_thetalist)
                new_thetalist = [round(theta,3) for theta in new_thetalist]
                print(f'[DEBUG] Current thetalist (deg) = {curr_thetalist}') 
                print(f'[DEBUG] Commanded thetalist (deg) = {new_thetalist}')  
                print(f'---------------------------------------------------------------------')
                
                # set new joint angles
                robot.set_joint_values(new_thetalist, duration=dt, radians=False)

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


