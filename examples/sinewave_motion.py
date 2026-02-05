# main.py
"""
Main Application Script
----------------------------
Coordinates gamepad input and robot control.
"""

import time
import traceback
import numpy as np

from funrobo_hiwonder.core.hiwonder import HiwonderRobot



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()
        
        OFFSETS = [0, 0, 90, -30, 0, 0]
        AMPS    = [25, 20, 20, 25, 30, 30]
        PHASES  = [0.0, 0.7, 1.4, 2.1, 2.8, 3.5]  # radians
        DURATION = 20.0              # seconds total run
        PERIOD = 6.0                 # seconds per cycle (smooth)   

        while True:

            t_start = time.time()
            now = time.time()
            t = now - t0
            if t >= DURATION:
                break

            # Smooth sine pattern
            angles = []
            for i in range(6):
                theta = OFFSETS[i] + AMPS[i] * np.sin(2.0 * np.pi * t / PERIOD + PHASES[i])
                angles.append(theta)

            robot.set_joint_values(angles, duration=dt)


            print(f'[DEBUG]  [Time: {time.time()-t0}] Joint values are: {robot.get_joint_values()}')


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


