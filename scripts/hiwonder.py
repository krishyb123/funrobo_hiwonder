# hiwonder.py
"""
Hiwonder Robot Controller
-------------------------
Handles the control of the mobile base and 5-DOF robotic arm using commands received from the gamepad.
"""

import time
import numpy as np
import threading
import traceback
from ros_robot_controller_sdk import Board
from gamepad_control import GamepadControl
from bus_servo_control import *

import utils as ut

# Robot base constants
WHEEL_RADIUS = 0.047  # meters
BASE_LENGTH_X = 0.096  # meters
BASE_LENGTH_Y = 0.105  # meters



class HiwonderRobot:

    def __init__(self):
        self.board = Board()
        self.board.enable_reception()
        self.board_lock = threading.Lock()

        self.joint_values = [0.0] * 6
        self.joint_limits = [
            [-120, 120], [-90, 90], [-120, 120],
            [-100, 100], [-90, 90], [-120, 30]
        ]
        self.home_position = [0, 0, 90, -30, 0, 0] # degrees

        self.read_hz = 5
        self.shutdown_event = threading.Event()
        self.read_error = None
        self.joint_lock = threading.Lock()

        self.read_thread = threading.Thread(target=self.read_joint_values, daemon=True)
        self.read_thread.start()

        self.gamepad = GamepadControl()
        self.gamepad_thread = threading.Thread(target=self.gamepad.monitor_gamepad, daemon=True)
        self.gamepad_thread.start()

        self.initialize_robot()
    
 

    def set_joint_values(self, thetalist: list, duration=1, radians=False):
        """Moves all arm joints to the given angles.

        Args:
            thetalist (list): Target joint angles in degrees.
            duration (float): Movement duration in seconds.
        """
        if len(thetalist) != 6:
            raise ValueError("Provide 6 joint angles.")
        
        if radians:
            thetalist = [np.rad2deg(theta) for theta in thetalist]

        thetalist = self.enforce_joint_limits(thetalist)
        #self.joint_values = thetalist # updates joint_values with commanded thetalist
        thetalist = self.remap_joints(thetalist) # remap the joint values from software to hardware
        
        positions = []
        for joint_id, theta in enumerate(thetalist, start=1):
            pulse = self.angle_to_pulse(theta)
            positions.append([joint_id, pulse])
        with self.board_lock:
            self.board.bus_servo_set_position(duration, positions)


    def read_joint_values(self):
        try:
            while not self.shutdown_event.is_set():
                t = time.time()
                res = [self.read_joint_value(i+1) for i in range(len(self.joint_values))]
                res = self.remap_joints(res)

                # convert pulses -> angles, keep last values on None
                with self.joint_lock:
                    prev = self.joint_values.copy()

                for i in range(len(res)):
                    with self.joint_lock:
                        if res[i] is None:
                            self.joint_values[i] = prev[i]
                        else:
                            self.joint_values[i] = self.pulse_to_angle(res[i][0])

                # with self.joint_lock:
                #     self.joint_values = res

                time.sleep(1 / self.read_hz)

                dt = time.time() - t
                if dt > (10/self.read_hz):
                    print("[WARN] reader slow:", dt, "raw:", res)
                    raise RuntimeError("Read thread is damaged... restart!")

        except Exception as e:
            self.read_error = e
            self.shutdown_event.set()


    def read_joint_value(self, joint_id: int):
        """ Gets the joint angle """
        max_count = 20 
        for _ in range(max_count):
            with self.board_lock:
                res = self.board.bus_servo_read_position(joint_id)
            if res is not None:
                return res
            time.sleep(0.01)
        return None


    def angle_to_pulse(self, x: float):
        """ Converts degrees to servo pulse value """
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -120, 120
        return int((x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min)


    def pulse_to_angle(self, x: float):
        """ Converts servo pulse value to degrees """
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -120, 120
        return round((x - hw_min) * (joint_max - joint_min) / (hw_max - hw_min) + joint_min, 2)


    def get_joint_values(self):
        """ Returns all the joint angle values """
        with self.joint_lock:
            return self.joint_values.copy()
    

    def enforce_joint_limits(self, thetalist: list) -> list:
        """Clamps joint angles within their hardware limits.

        Args:
            thetalist (list): List of target angles.

        Returns:
            list: Joint angles within allowable ranges.
        """
        return [np.clip(theta, *limit) for theta, limit in zip(thetalist, self.joint_limits)]


    def initialize_robot(self):
        with self.board_lock:
            self.board.set_buzzer(2400, 0.1, 0.9, 1)
        time.sleep(1)
    
        self.move_to_home_position()

        time.sleep(1)
        print(f'------------------- System is now ready!------------------- \n')


    def move_to_home_position(self):
        print(f'Moving to home position...')
        self.set_joint_values(self.home_position, duration=1)
        time.sleep(2.0)

        print(f'Arrived at home position: {self.home_position} \n')
        
        with self.joint_lock:
            self.joint_values = self.home_position.copy()
        
        time.sleep(1.0)


    def shutdown_robot(self):
        print("\n[INFO] Shutting down the robot safely...")
        
        self.set_joint_values(self.home_position, duration=2)
        time.sleep(1.5)  # Allow time for servos to reposition

        self.disable_servos()
        self.shutdown_event.set()

        print("[INFO] Closing hardware interfaces...")    

        with self.board_lock:
            try:
                self.board.enable_reception(False)
            except Exception:
                pass
            try:
                self.board.port.close()
            except Exception:
                pass

        print("[INFO] Shutdown complete. Safe to power off.")


    def disable_servos(self):
        with self.board_lock:
            for joint_id in range(1, len(self.joint_values) + 1):
                self.board.bus_servo_enable_torque(joint_id, 0)


    def remap_joints(self, thetalist: list):
        """Reorders angles to match hardware configuration.

        Args:
            thetalist (list): Software joint order.

        Returns:
            list: Hardware-mapped joint angles.

        Note: Joint mapping for hardware
            HARDWARE - SOFTWARE
            joint[0] = gripper/EE
            joint[1] = joint[5] 
            joint[2] = joint[4] 
            joint[3] = joint[3] 
            joint[4] = joint[2] 
            joint[5] = joint[1] 
        """
        return thetalist[::-1]
        
