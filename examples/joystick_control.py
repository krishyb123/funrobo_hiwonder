
import time
import traceback

from funrobo_hiwonder.core.hiwonder import HiwonderRobot



def joystick_control(robot, dt, new_joint_values):
    # get the latest gamepad command
    cmd = robot.gamepad.cmdlist[-1]

    # ----------------------------------------------------------------------
    # Arm joint control
    # ----------------------------------------------------------------------

    max_rate = 400  # 400 x 0.1 = 40 deg/s
    
    new_joint_values[0] += dt * max_rate * cmd.arm_j1
    new_joint_values[1] += dt * max_rate * cmd.arm_j2
    new_joint_values[2] += dt * max_rate * cmd.arm_j3
    new_joint_values[3] += dt * max_rate * cmd.arm_j4
    new_joint_values[4] += dt * max_rate * cmd.arm_j5
    new_joint_values[5] += dt * max_rate * cmd.arm_ee

    new_joint_values = robot.enforce_joint_limits(new_joint_values)
    new_joint_values = [round(theta,3) for theta in new_joint_values]

    print(f'[DEBUG] Commanded joint angles: [j1, j2, j3, j4, j5, ee]: {new_joint_values}')
    print(f'-------------------------------------------------------------------------------------\n')    
    
    # set new joint angles
    robot.set_joint_values(new_joint_values, duration=dt, radians=False)

    # ----------------------------------------------------------------------
    # base veocity control
    # ----------------------------------------------------------------------

    vx, vy, w = cmd.base_vx, cmd.base_vy, cmd.base_w

    # Compute wheel speeds
    w0 = (vx - vy - w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius
    w1 = (vx + vy + w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius
    w2 = (vx + vy - w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius
    w3 = (vx - vy + w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius

    robot.set_wheel_speeds([w0, w1, w2, w3])



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        
        control_hz = 20 
        dt = 1 / control_hz

        joint_values = robot.get_joint_values().copy()

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                joystick_control(robot, dt, joint_values)


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
