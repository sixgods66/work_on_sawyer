#! /home/znfs/pyenv_pyrobot_python2/bin/python

from robot_utils import *
import tf
import yaml
from geometry_msgs.msg import Pose

param_file = '/home/znfs/project_ws/intera/src/iros2021/files/kinect_calibration.yaml'


def quaternion_to_trans_matrix(x, y, z, w, trans_x, trans_y, trans_z):
    x_2, y_2, z_2 = x * x, y * y, z * z
    xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
    # origin_rotation_matrix        [1 - 2 * y_2 - 2 * z_2,  2 * xy + 2 * wz,        2 * xz - 2 * wy,
    #                                2 * xy - 2 * wz,        1 - 2 * x_2 - 2 * z_2,  2 * yz + 2 * wx,
    #                                2 * xz + 2 * wy,        2 * yz - 2 * wx,        1 - 2 * x_2 - 2 * y_2]
    translation_matrix = np.array([1 - 2 * y_2 - 2 * z_2, 2 * xy - 2 * wz, 2 * xz + 2 * wy, trans_x,
                                   2 * xy + 2 * wz, 1 - 2 * x_2 - 2 * z_2, 2 * yz - 2 * wx, trans_y,
                                   2 * xz - 2 * wy, 2 * yz + 2 * wx, 1 - 2 * x_2 - 2 * y_2, trans_z,
                                   0, 0, 0, 1
                                   ])
    return translation_matrix.reshape((4, 4))


def resolve_ext(yaml_data):
    orientation = yaml_data['rot']
    trans = yaml_data['trans']
    trans_matrix = quaternion_to_trans_matrix(orientation[0], orientation[1], orientation[2], orientation[3],
                                              trans[0], trans[1], trans[2])
    return trans_matrix


# lookup tf transform between two frames
def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))

    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)
    source_target = tf.transformations.compose_matrix(translate=trans, angles=euler)
    return trans, rot, source_target


def resolve(trans, rot):
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]

    return pose


def main():
    rospy.init_node('publish_static_tf')
    pub = rospy.Publisher('/base2ee', Pose, queue_size=1)

    file = open(param_file, 'r')
    file_data = file.read()

    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # loop rate
    rate = rospy.Rate(100)

    k_extrin = resolve_ext(yaml.safe_load(file_data))
    rospy.loginfo("start to pub /base2ee tf topic")
    while not rospy.is_shutdown():
        trans, rot, source_target = lookupTransform(tf_listener, '/base', '/right_gripper_tip')
        base2eePose = resolve(trans, rot)
        pub.publish(base2eePose)


if __name__ == "__main__":
    main()
