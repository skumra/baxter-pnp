import time

import numpy as np

from hardware.robot import Robot
from utils.transforms import get_pose


class Calibration:
    def __init__(self,
                 tool_orientation=[-np.pi / 2, 0, 0]
                 ):
        self.tool_orientation = tool_orientation
        self.robot = Robot()
        
    def run(self):
        # Connect to robot
        self.robot.connect()

        # Move robot to home pose
        print('Moving to start position...')
        self.robot.go_home()
        self.robot.open_gripper()

        # Move robot to each calibration point in workspace
        print('Collecting data...')
        while True:
            if not np.load("move_completed.npy"):
                tool_position = np.load("tool_position.npy")
                print('Moving to tool position: ', tool_position)
                pose = get_pose(position=tool_position, orientation=self.tool_orientation)
                self.robot.move_to(pose)
                np.save("move_completed.npy", 1)
            else:
                time.sleep(0.1)
