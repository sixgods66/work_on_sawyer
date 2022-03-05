# coding=utf-8

import rospy
import math

import moveit_commander
import moveit_msgs.msg

from math import pi
from scipy.spatial.transform import Rotation as R
from moveit_commander.conversions import pose_to_list

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION

import numpy as np
import geometry_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from voice.msg import VoiceMsg

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True





class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


class RobotInit(MoveGroupPythonIntefaceTutorial):
    def __init__(self, speed=0.3):
        super(RobotInit, self).__init__()
        self.rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = self.rs.state().enabled
        self.rp = intera_interface.RobotParams()
        self.valid_limbs = self.rp.get_limb_names()
        self.limb = intera_interface.Limb(self.valid_limbs[0])
        self.joint_names = self.limb.joint_names()
        self.limb.set_joint_position_speed(speed)
        self.gripper = None
        self.original_deadzone = None
        self._rate = 500.0
        self.rate = rospy.Rate(self._rate)
        self.move_group.set_max_acceleration_scaling_factor(speed)
        self.move_group.set_max_velocity_scaling_factor(speed)
        self.cameras = intera_interface.Cameras()
        self.right_camera_name = "right_hand_camera"
        sub = rospy.Subscriber("voice", VoiceMsg, self.voice_callback, queue_size=1, buff_size=1, tcp_nodelay=True)

        self.response = None
        self.trans_matrix = None

        try:
            self.gripper = intera_interface.Gripper(self.valid_limbs[0] + '_gripper')
        except (ValueError, OSError) as e:
            rospy.logerr("Could not detect an electric gripper attached to the robot.")
            self.clean_shutdown()

    def set_detect_result(self, response):
        self.response = response

    def set_transmatrix(self, transmatrix):
        self.trans_matrix = transmatrix

    def change_speed(self, speed):  # 改变关节的速度（0-1）
        self.limb.set_joint_position_speed(speed)

    def clean_shutdown(self):  # 设置夹爪的死区
        if self.gripper and self.original_deadzone:
            self.gripper.set_dead_zone(self.original_deadzone)
        print("Exiting example.")

    def set_camera_gain(self, gain_value, camera_name="right_hand_camera"):  # 头部相机为 head_camera
        cameras = self.cameras
        if cameras.set_gain(camera_name, gain_value):
            rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(camera_name)))

    # 控制相机的曝光
    def set_camera_exposure(self, exposure_value, camera_name="right_hand_camera"):  # 头部相机为 head_camera
        cameras = self.cameras
        if cameras.set_exposure(camera_name, exposure_value):
            rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(camera_name)))

    def go_to_camera_position(self):
        joints_command = {'right_j0': -2.3301435546875, 'right_j1': -3.047884765625, 'right_j2': 1.4562392578125,
                          'right_j3': -1.4219619140625, 'right_j4': 1.7223046875, 'right_j5': 0.00515625,
                          'right_j6': 3.3662646484375}

        self.limb.move_to_joint_positions(joints_command)

    def control_gripper(self, cmd_pos, dead_zone=0.):  # 0~100 close~open
        self.gripper.set_dead_zone(dead_zone)
        # cmd_pos = max(min(self.gripper.get_position() + offset_pos, self.gripper.MAX_POSITION),
        #               self.gripper.MIN_POSITION)
        cmd_pos /= 2000.
        self.gripper.set_position(cmd_pos)

    def get_gripper_position(self):
        return self.gripper.get_position()

    def get_gripper_force(self):
        return self.gripper.get_force()

    def get_gripper_object_weight(self):  # right_hand:position, orientation
        return self.gripper.get_object_weight()

    def voice_callback(self, msg):
        """
                rect  round  hex  rect-b  round-b  hex-b
        cl       0      1     2     3       4       5
        v-order  2      3     4
        @param msg:
        @return:
        """
        print('receive command is: {}.'.format(msg.order))

        if msg.order == 2:
            self.assembly_proc(0)
        elif msg.order == 3:
            self.assembly_proc(1)
        elif msg.order == 4:
            self.assembly_proc(2)

    def moveit_move(self, position, orientation=None):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        if orientation is not None:
            pose_goal.orientation.x = orientation[0]
            pose_goal.orientation.y = orientation[1]
            pose_goal.orientation.z = orientation[2]
            pose_goal.orientation.w = orientation[3]
        else:
            pose_goal.orientation.x = 0.707
            pose_goal.orientation.y = -0.707
            pose_goal.orientation.z = 0.0
            pose_goal.orientation.w = 0.0

        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def sawyer_ik_move(self, position, orientation=None):
        ns = "ExternalTools/" + self.valid_limbs[0] + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        if orientation == None:
            orientation_ = Quaternion(x=0.707, y=-0.707, z=0.0, w=0.0, )
        else:
            orientation_ = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3], )

        poses = {'right': PoseStamped(header=hdr,
                                      pose=Pose(position=Point(
                                          x=position[0], y=position[1], z=position[2], ),
                                          orientation=orientation_,
                                      ), ), }
        ikreq.pose_stamp.append(poses[self.valid_limbs[0]])
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append('right_hand')

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        if resp.result_type[0] > 0:
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp.result_type[0], 'None')
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                          (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(list(zip(resp.joints[0].name, resp.joints[0].position)))

            rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
            rospy.loginfo("------------------")
            # rospy.loginfo("Response Message:\n%s", resp)
            self.limb.move_to_joint_positions(limb_joints)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            rospy.logerr("Result Error %d", resp.result_type[0])
            return False

    def get_cur_pos(self):
        return intera_interface.Limb(self.valid_limbs[0]).endpoint_pose()

    def adjust_det_pose(self, detect_position, bias):
        detect_position[0] += bias[0]
        detect_position[1] += bias[1]
        detect_position[2] += bias[2]

        return detect_position

    def assembly_proc(self, cl):
        if self.response is None or self.trans_matrix is None:
            print('detect result is not prepared')
            return

        detect_orientation = [0.0, 1.0, 0.0, 0.0]
        response = self.response

        workpiece_position = self.trans_matrix.dot([response[cl].pos_x, response[cl].pos_y, response[cl].pos_z, 1])
        print('workpiece: ', cl, response[cl].pos_x, response[cl].pos_y, response[cl].pos_z, response[cl].cl)
        print(workpiece_position)

        detect_position = np.array([workpiece_position[0], workpiece_position[1], workpiece_position[2]])
        detect_position = self.adjust_det_pose(detect_position, [-0.008, 0.015, 0.0])
        self.moveit_move(detect_position, detect_orientation)

        detect_position = self.adjust_det_pose(detect_position, [0., 0., -0.07])
        self.moveit_move(detect_position, detect_orientation)
        self.control_gripper(0)

        # detect_position = base_position
        detect_position = self.adjust_det_pose(detect_position, [0., 0., 0.40])
        self.moveit_move(detect_position, detect_orientation)
        print("*** grasp workpiece {} is finished".format(cl))

        base_cl = cl + 3

        position = self.trans_matrix.dot([response[base_cl].pos_x, response[base_cl].pos_y, response[base_cl].pos_z, 1])
        base_detect_position = np.array([position[0], position[1], position[2]])

        if base_cl == 3:
            base_detect_position = self.adjust_det_pose(base_detect_position, [-0.008, 0.01, 0.05])
            self.moveit_move(base_detect_position, detect_orientation)

            base_detect_position = self.adjust_det_pose(base_detect_position, [0., 0., -0.05])
            self.moveit_move(base_detect_position, detect_orientation)

            self.change_speed(0.02)
            base_detect_position = self.adjust_det_pose(base_detect_position, [-0.002, 0.005, -0.015])
            self.moveit_move(base_detect_position, detect_orientation)

        elif base_cl == 4:
            base_detect_position = self.adjust_det_pose(base_detect_position, [-0.008, 0.01, 0.05])
            self.moveit_move(base_detect_position, detect_orientation)

            base_detect_position = self.adjust_det_pose(base_detect_position, [0., 0., -0.05])
            self.moveit_move(base_detect_position, detect_orientation)

            self.change_speed(0.02)
            base_detect_position = self.adjust_det_pose(base_detect_position, [-0.0015, 0.002, -0.015])
            self.moveit_move(base_detect_position, detect_orientation)

        elif base_cl == 5:
            base_detect_position = self.adjust_det_pose(base_detect_position, [-0.008, 0.01, 0.05])
            self.moveit_move(base_detect_position, detect_orientation)

            base_detect_position = self.adjust_det_pose(base_detect_position, [0., 0., -0.05])
            self.moveit_move(base_detect_position, detect_orientation)

            self.change_speed(0.02)
            base_detect_position = self.adjust_det_pose(base_detect_position, [-0.002, 0.002, -0.015])
            self.moveit_move(base_detect_position, detect_orientation)

        self.control_gripper(100)
        print("*** put in base {} is finished".format(base_cl))
        self.change_speed(0.3)
        self.moveit_move(np.array([0.251, -0.154, 0.599]), detect_orientation)



if __name__ == "__main__":
    rospy.init_node("control_node", anonymous=True)
