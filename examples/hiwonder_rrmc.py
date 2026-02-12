# main.py
"""
Main Application Script
----------------------------
Example code for the MP1 RRMC implementation
"""

import time
import traceback

from funrobo_hiwonder.core.hiwonder import HiwonderRobot

from projects.mp1.five_dof import FiveDOFRobot
import funrobo_kinematics.core.utils as ut



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        model = FiveDOFRobot()
        
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                cmd = robot.gamepad.cmdlist[-1]

                if cmd.arm_home:
                    robot.move_to_home_position()

                curr_joint_values = robot.get_joint_values()

                vel = [cmd.arm_vx, cmd.arm_vy, cmd.arm_vz]
                new_joint_values = model.calc_velocity_kinematics(curr_joint_values, vel)

                # set new joint angles
                robot.set_joint_values(new_joint_values, duration=dt, radians=False)

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


