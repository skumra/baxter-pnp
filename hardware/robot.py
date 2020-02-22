import struct

import baxter_interface
import rospy
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
)
from std_msgs.msg import (
    Header,
)


class Robot:
    def __init__(self, workspace_limits=None, limb='right', verbose=True):
        self.workspace_limits = workspace_limits
        self.limb = limb
        self._verbose = verbose

        # Initialize ros node
        rospy.init_node("pick_and_place")

        self._limb = baxter_interface.Limb(self.limb)
        self._gripper = baxter_interface.Gripper(self.limb)

        self._iksvc = None

    def _ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if resp_seeds[0] != resp.RESULT_INVALID:
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(seed_str))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints    

    def connect(self):
        """ Establish connection with the robot """
        # verify robot is enabled
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        if not rs.state().enabled:
            print("Enabling robot... ")
            rs.enable()
        print("Robot enabled... ")

        ns = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

    def close_gripper(self):
        """ Close robot gripper """
        self._gripper.close()
        rospy.sleep(1.0)

    def open_gripper(self):
        """ Open robot gripper """
        self._gripper.open()
        rospy.sleep(1.0)

    def calibrate_gripper(self):
        """ Calibrate gripper """
        self._gripper.calibrate()

    def move_to(self, pose, timeout=15.0):
        """ Move robot in cartesian space """
        joint_angles = self._ik_request(pose)
        self.move_joints(joint_angles, timeout)

    def move_joints(self, joint_angles, timeout):
        """ Move robot in joint space """
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles, timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def current_pose(self):
        """ Return current pose from endpoint """
        current_pose = self._limb.endpoint_pose()
        return current_pose

    def go_home(self):
        """ Move robot to home position """
        self._limb.move_to_neutral()

    def go_to_dofs(self, angles):
        """ Move robot to angles """
        joint_angles = dict(zip(self._limb._joint_names[self.limb], angles))
        self.move_joints(joint_angles)

    def get_force(self):
        """ Returns force at end effector """
        return self._limb.endpoint_effort()['force']

