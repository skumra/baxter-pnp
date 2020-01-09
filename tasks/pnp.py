import copy
import os
import time

import numpy as np
from geometry_msgs.msg import Pose

from hardware.robot import Robot
from utils.transforms import rotate_pose_msg_by_euler_angles, get_pose


class PickAndPlace:
    def __init__(
            self,
            hover_distance,
            place_position
    ):
        self._hover_distance = hover_distance  # in meters
        self.place_position = place_position

        self.robot = Robot()

        homedir = os.path.join(os.path.expanduser('~'), "grasp-comms")
        self.grasp_request = os.path.join(homedir, "grasp_request.npy")
        self.grasp_available = os.path.join(homedir, "grasp_available.npy")
        self.grasp_pose = os.path.join(homedir, "grasp_pose.npy")

    def _approach(self, pose):
        """
        Approach with a pose the hover-distance above the requested pose
        """
        approach = copy.deepcopy(pose)
        approach.position.z = approach.position.z + self._hover_distance
        self.robot.move_to(approach)

    def _servo_to_pose(self, pose):
        """
        Servo down to pose
        """
        self.robot.move_to(pose)

    def _retract(self):
        """
        Retract up from current pose
        """
        # retrieve current pose from endpoint
        current_pose = self.robot.current_pose()
        pose = Pose()
        pose.position.x = current_pose['position'].x
        pose.position.y = current_pose['position'].y
        pose.position.z = current_pose['position'].z + self._hover_distance
        pose.orientation.x = current_pose['orientation'].x
        pose.orientation.y = current_pose['orientation'].y
        pose.orientation.z = current_pose['orientation'].z
        pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self.robot.move_to(pose)

    def pick(self, pose):
        """
        Pick from given pose
        """
        # open the gripper
        self.robot.open_gripper()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.robot.close_gripper()
        # retract to clear object
        self._retract()

    def place(self, place_position):
        """
        Place to given pose
        """
        # Calculate pose from place position
        pose = get_pose(position=place_position)

        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.robot.open_gripper()
        # retract to clear object
        self._retract()

    def run(self):
        # Connect to robot
        self.robot.connect()

        # Initialize grasp request and grasp available
        np.save(self.grasp_request, 0)
        np.save(self.grasp_available, 0)

        while True:
            # Move robot to home pose
            print('Moving to start position...')
            self.robot.go_home()
            self.robot.open_gripper()

            # Get the grasp pose
            np.save(self.grasp_request, 1)
            print('Waiting for grasp pose...')
            while not np.load(self.grasp_available):
                time.sleep(0.1)
            gp = np.load(self.grasp_pose)
            np.save(self.grasp_available, 0)

            # Calculate grasp pose
            grasp_pose = get_pose(position=gp[:3])

            # Apply grasp angle from model output
            grasp_pose = rotate_pose_msg_by_euler_angles(grasp_pose, 0.0, 0.0, gp[3])

            # Perform pick
            print('Picking from ', grasp_pose)
            self.pick(grasp_pose)

            # Perform place
            print('Placing to ', self.place_position)
            self.place(self.place_position)
