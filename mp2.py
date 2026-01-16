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
import utils as ut

WAYPOINTS = [[0.2, 0.1, 0.03],
             [0.2, 0.0, 0.03],
             [0.2, 0.1, 0.03]]

JOINT_WAYPOINTS = [[25.0, -77.0, 4.8, -83.2, 0.0, 0.0],
                   [1.0, -77.0, 4.8, -94.2, 0.0, 0.0],
                   [-25.0, -77.0, 4.8, -83.2, 0.0, 0.0]]

HOME_POSITION = [0, 0, 90, -30, 0, 0]

# Initialize components
robot = HiwonderRobot()
model = FiveDOFRobot()


def follow_waypoint():
    time.sleep(2)
    robot.set_joint_values(JOINT_WAYPOINTS[0], duration=1, radians=False)
    position = model.solve_forward_kinematics(robot.get_joint_values())
    print(f'Positions: {[round(p,3) for p in position]}')
    time.sleep(2)
    
    robot.set_joint_values(JOINT_WAYPOINTS[1], duration=1, radians=False)
    position = model.solve_forward_kinematics(robot.get_joint_values())
    print(f'Positions: {[round(p,3) for p in position]}')
    time.sleep(2)

    robot.set_joint_values(JOINT_WAYPOINTS[2], duration=1, radians=False)
    position = model.solve_forward_kinematics(robot.get_joint_values())
    print(f'Positions: {[round(p,3) for p in position]}')

    print(f'---------------------------------------------------------------------')



def follow_positions():
    time.sleep(2)
    ee = ut.EndEffector(x=WAYPOINTS[0][0], 
                        y=WAYPOINTS[0][1],
                        z=WAYPOINTS[0][2])
    thetalist = model.solve_inverse_kinematics(ee, robot.get_joint_values(), soln=1)
    print(f'Joint locations: {[round(th*180/3.142,3) for th in thetalist]}')

    time.sleep(2)
    ee = ut.EndEffector(x=WAYPOINTS[1][0], 
                        y=WAYPOINTS[1][1],
                        z=WAYPOINTS[1][2])
    thetalist = model.solve_inverse_kinematics(ee, robot.get_joint_values())
    print(f'Joint locations: {[round(th*180/3.142,3) for th in thetalist]}')

    print(f'---------------------------------------------------------------------')



class FollowWaypointFSM():
    def __init__(self, dt=0.05):
        self.stage = 0
        self.t = 0.0
        self.dt = dt
        self.T = 3 # seconds

    def step(self):
        if self.stage == 0:
            start_pos = HOME_POSITION
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*JOINT_WAYPOINTS[0][i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage = 1
                self.t = 0.0
        
        elif self.stage == 1:
            start_pos = JOINT_WAYPOINTS[0]
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*JOINT_WAYPOINTS[1][i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage = 2
                self.t = 0.0

        elif self.stage == 2:
            start_pos = JOINT_WAYPOINTS[1]
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*JOINT_WAYPOINTS[2][i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage = 3
                self.t = 0.0
        
        elif self.stage == 3:
            start_pos = JOINT_WAYPOINTS[2]
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*HOME_POSITION[i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage = 0
                self.t = 0.0

        print(f'Stage: {self.stage} theta: {theta} t: {round(self.t,3)})')
        robot.set_joint_values(theta, duration=self.dt, radians=False)





def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:
       
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()

        fsm = FollowWaypointFSM(dt=dt)

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                # follow_waypoint()
                # follow_positions()  # <-- not working yet
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


