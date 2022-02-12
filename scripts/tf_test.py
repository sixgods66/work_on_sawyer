#! /home/znfs/pyenv_pyrobot_python2/bin/python

import rospy
from geometry_msgs.msg import Pose


def main():
    rospy.init_node('tf_test')
    data = rospy.wait_for_message('/base2ee', Pose, timeout=4.0)
    print(data)


if __name__ == '__main__':
    main()

