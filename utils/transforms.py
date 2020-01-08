from geometry_msgs.msg import *
from tf.transformations import *


def get_pose(position, orientation=[1, 0, 0, 0]):
    grasp_pose = Pose()
    grasp_pose.position.x = position[0]
    grasp_pose.position.y = position[1]
    grasp_pose.position.z = position[2]
    grasp_pose.orientation.x = orientation[0]
    grasp_pose.orientation.y = orientation[1]
    grasp_pose.orientation.z = orientation[2]
    grasp_pose.orientation.w = orientation[3]

    return grasp_pose


def matrix_from_point_msg(point):
    """
    ## @brief Get a translation matrix from a geometry_msgs/Point
    ## @param point geometry_msgs/Point to turn into matrix
    """
    return translation_matrix((point.x, point.y, point.z))


def matrix_from_quaternion_msg(quaternion):
    """
    ## @brief Get a rotation matrix from a geometry_msgs/Quaternion
    ## @param quaternion geometry_msgs/Quaternion to turn into matrix
    """
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_matrix(q)


def matrix_from_pose_msg(pose):
    """
    ## @brief Get a transformation matrix from a geometry_msgs/Pose
    ## @param pose geometry_msgs/Pose to turn into matrix
    """
    t = matrix_from_point_msg(pose.position)
    r = matrix_from_quaternion_msg(pose.orientation)
    return concatenate_matrices(t, r)


def point_msg_from_matrix(transformation):
    """
    ## @brief Get a geometry_msgs/Point from a transformation matrix
    ## @param transformation The matrix to convert to a point
    """
    msg = Point()
    msg.x = transformation[0][3]
    msg.y = transformation[1][3]
    msg.z = transformation[2][3]
    return msg


def quaternion_msg_from_matrix(transformation):
    """
    ## @brief Get a geometry_msgs/Quaternion from a transformation matrix
    ## @param transformation The matrix to convert to a quaternion
    """
    q = quaternion_from_matrix(transformation)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def pose_msg_from_matrix(transformation):
    """
    ## @brief Get a geometry_msgs/Pose from a transformation matrix
    ## @param transformation The matrix to convert to a pose
    """
    msg = Pose()
    msg.position = point_msg_from_matrix(transformation)
    msg.orientation = quaternion_msg_from_matrix(transformation)
    return msg


def translate_pose_msg(pose, x, y, z):
    """
    ## @brief Translate a geometry_msgs/Pose
    ## @param pose The pose to translate
    ## @param x The displacement in X coordinate axis
    ## @param y The displacement in Y coordinate axis
    ## @param z The displacement in Z coordinate axis
    """
    initial = matrix_from_pose_msg(pose)
    transform = translation_matrix((x,y,z))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_by_euler_angles(pose, r, p, y):
    """
    ## @brief Rotate a geometry_msgs/Pose
    ## @param pose The pose to rotate
    ## @param r The roll
    ## @param p The pitch
    ## @param y The yaw
    """
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_about_origin(pose, r, p, y):
    """
    ## @brief Rotate a geometry_msgs/Pose
    ## @param pose The pose to rotate
    ## @param r The roll
    ## @param p The pitch
    ## @param y The yaw
    """
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(transform, initial))
