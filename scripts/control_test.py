#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy, sys
import tf
import time
import numpy as np
import moveit_commander
from math import pi
from moveit_commander import PlanningSceneInterface
from moveit_commander.conversions import pose_to_list

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import moveit_msgs.msg
from workpiece_proc.msg import Object
from workpiece_proc.srv import imageproc, imageprocRequest, imageprocResponse
from robot_utils import RobotInit

from voice.msg import VoiceMsg
from voice.srv import VoiceSrv, VoiceSrvRequest, VoiceSrvResponse

class DetectInfo:
    def __init__(self):
        self.client = rospy.ServiceProxy('imageProc', imageproc)
        self.tf_listener = tf.TransformListener()

    def get_detect_info(self):
        self.client.wait_for_service()
        request = imageprocRequest()
        request.c = 1
        response = self.client.call(request)
        if isinstance(response, imageprocResponse):
            print("saved")
            return response.obj
        else:
            print('error instance')
            return None

    def get_base_to_cam_t(self):
        self.tf_listener.waitForTransform('/base', '/kinect2_link', rospy.Time(0), rospy.Duration(5), None)
        (self.trans, self.rot) = self.tf_listener.lookupTransform('/base', '/kinect2_link', rospy.Time(0))
        self.trans_matrix = self.quaternion_to_trans_matrix(self.rot[0], self.rot[1], self.rot[2], self.rot[3],
                                                            self.trans[0], self.trans[1], self.trans[2])
        return self.trans_matrix

    def quaternion_to_trans_matrix(self, x, y, z, w, trans_x, trans_y, trans_z):
        x_2, y_2, z_2 = x * x, y * y, z * z
        xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
        # origin_rotation_matrix        [1 - 2 * y_2 - 2 * z_2,  2 * xy + 2 * wz,        2 * xz - 2 * wy,
        #                                2 * xy - 2 * wz,        1 - 2 * x_2 - 2 * z_2,  2 * yz + 2 * wx,
        #                                2 * xz + 2 * wy,        2 * yz - 2 * wx,        1 - 2 * x_2 - 2 * y_2]
        self.translation_matrix = np.array([1 - 2 * y_2 - 2 * z_2, 2 * xy - 2 * wz, 2 * xz + 2 * wy, trans_x,
                                            2 * xy + 2 * wz, 1 - 2 * x_2 - 2 * z_2, 2 * yz - 2 * wx, trans_y,
                                            2 * xz - 2 * wy, 2 * yz + 2 * wx, 1 - 2 * x_2 - 2 * y_2, trans_z,
                                            0, 0, 0, 1
                                            ])
        return self.translation_matrix.reshape((4, 4))

def all_close(goal, actual, tolerance):
    """
      Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
      @param: goal       A list of floats, a Pose or a PoseStamped
      @param: actual     A list of floats, a Pose or a PoseStamped
      @param: tolerance  A float
      @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial:
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        #super(MoveGroupPythonIntefaceTutorial, self).__init__()
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)

        #rospy.init_node('move_group_python_interface_tutorial', anonymous=False)

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

    def go_to_joint_state(self):
        move_group = self.move_group
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi / 4
        joint_goal[2] = 0
        joint_goal[3] = -pi / 2
        joint_goal[4] = 0
        joint_goal[5] = pi / 3
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position, orientation):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

class GripperConnect:
    """
    Connects wrist button presses to gripper open/close commands.

    Uses the Navigator callback feature to make callbacks to connected
    action functions when the button values change.
    """

    def __init__(self, arm, lights=True):
        """
        @type arm: str
        @param arm: arm of gripper to control
        @type lights: bool
        @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm
        # inputs
        self._cuff = Cuff(limb=arm)
        # connect callback fns to signals
        self._lights = None
        if lights:
            self._lights = Lights()
            self._cuff.register_callback(self._light_action,
                                         '{0}_cuff'.format(arm))
        try:
            self._gripper = get_current_gripper_interface()
            self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

            if self._is_clicksmart:
                if self._gripper.needs_init():
                    self._gripper.initialize()
            else:
                if not (self._gripper.is_calibrated() or
                        self._gripper.calibrate() == True):
                    raise
            self._cuff.register_callback(self._close_action,
                                         '{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,
                                         '{0}_button_lower'.format(arm))

            rospy.loginfo("{0} Cuff Control initialized...".format(
                          self._gripper.name))
        except:
            self._gripper = None
            self._is_clicksmart = False
            msg = ("{0} Gripper is not connected to the robot."
                   " Running cuff-light connection only.").format(arm.capitalize())
            rospy.logwarn(msg)

    def _open_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper open triggered")
            #if self._is_clicksmart:
                #self._gripper.set_ee_signal_value('grip', False)
            #else:
            self._gripper.open()
            if self._lights:
                self._set_lights('red', False)
                self._set_lights('green', True)

    def _close_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
            #if self._is_clicksmart:
                #self._gripper.set_ee_signal_value('grip', True)
            #else:
            self._gripper.close()
            if self._lights:
                self._set_lights('green', False)
                self._set_lights('red', True)

    def _light_action(self, value):
        if value:
            rospy.logdebug("cuff grasp triggered")
        else:
            rospy.logdebug("cuff release triggered")
        if self._lights:
            self._set_lights('red', False)
            self._set_lights('green', False)
            self._set_lights('blue', value)

    def _set_lights(self, color, value):
        self._lights.set_light_state('head_{0}_light'.format(color), on=bool(value))
        self._lights.set_light_state('{0}_hand_{1}_light'.format(self._arm, color),
                                                                 on=bool(value))


if __name__ == "__main__":
    rospy.init_node('assemble_task', anonymous=True)
    # 初始化ROS节点
    det = DetectInfo()

    voice_order()

    detect_orientation = [0.0, 1.0, 0.0, 0.0]
    robot_control = RobotInit(0.3)
    mgpit = MoveGroupPythonIntefaceTutorial()
    #mgpit.go_to_joint_state()
    robot_control.control_gripper(100) #  0~100 close~open

    #robot_control.moveit_move(np.array([0.2600, 0, 0.489]), detect_orientation)
    robot_control.moveit_move(np.array([0.251, -0.154, 0.599]), detect_orientation)

    response = det.get_detect_info()
    response = sorted(response, key=lambda x: x.cl)
    trans_matrix = det.get_base_to_cam_t()

    #base_position = np.array([0.486779, 0.1109, 0.099])
    base_cylinder_position = np.array([0.4298, 0.004563, 0.101558])
    rospy.sleep(1)

    trans_matrix = det.get_base_to_cam_t()

    for i in range(len(response)/2):
        
        obj = response[i]
        position = trans_matrix.dot([response[i].pos_x, response[i].pos_y, response[i].pos_z, 1])
        print(i, response[i].pos_x, response[i].pos_y, response[i].pos_z, response[i].cl)
        print(position)

        cl = response[i].cl

        detect_position = np.array([position[0], position[1], position[2]])
        detect_position[2] -= 0.00
        detect_position[1] += 0.015
        detect_position[0] -= 0.008
        robot_control.moveit_move(detect_position, detect_orientation)
        
        detect_position[2] -= 0.07
        detect_position[1] += 0.00
        detect_position[0] += 0.00
        robot_control.moveit_move(detect_position, detect_orientation)
        robot_control.control_gripper(0)
        
        #detect_position = base_position
        detect_position[2] += 0.40
        detect_position[1] += 0.00
        detect_position[0] += 0.00
        robot_control.moveit_move(detect_position, detect_orientation)
        
        for j in range(len(response)/2,len(response)):
            
            if j == i + len(response)/2:
                obj = response[j]
                position = trans_matrix.dot([response[j].pos_x, response[j].pos_y, response[j].pos_z, 1])
                detect_position = np.array([position[0], position[1], position[2]])
                print(j, response[j].pos_x, response[j].pos_y, response[j].pos_z, response[j].cl)
                print(position)
                if cl == 0:
                    detect_position[2] += 0.05
                    detect_position[1] += 0.01
                    detect_position[0] -= 0.008
                    robot_control.moveit_move(detect_position, detect_orientation)
    
                    detect_position[2] -= 0.05
                    detect_position[1] += 0.00
                    detect_position[0] -= 0.00
                    robot_control.moveit_move(detect_position, detect_orientation)

                    robot_control = RobotInit(0.02)
                    detect_position[2] -= 0.015
                    detect_position[1] += 0.005
                    detect_position[0] -= 0.002
                    robot_control.moveit_move(detect_position, detect_orientation)

                elif cl == 1:
                    detect_position[2] += 0.05
                    detect_position[1] += 0.01
                    detect_position[0] -= 0.008
                    robot_control.moveit_move(detect_position, detect_orientation)

                    detect_position[2] -= 0.05
                    detect_position[1] += 0.00
                    detect_position[0] -= 0.00
                    robot_control.moveit_move(detect_position, detect_orientation)
                    
                    robot_control = RobotInit(0.02)
                    detect_position[2] -= 0.015
                    detect_position[1] += 0.002
                    detect_position[0] -= 0.0015
                    robot_control.moveit_move(detect_position, detect_orientation)

                elif cl == 2:
                    detect_position[2] += 0.05
                    detect_position[1] += 0.01
                    detect_position[0] -= 0.008
                    robot_control.moveit_move(detect_position, detect_orientation)

                    detect_position[2] -= 0.05
                    detect_position[1] += 0.00
                    detect_position[0] -= 0.00
                    robot_control.moveit_move(detect_position, detect_orientation)
                    
                    robot_control = RobotInit(0.02)
                    detect_position[2] -= 0.015
                    detect_position[1] += 0.002
                    detect_position[0] -= 0.002
                    robot_control.moveit_move(detect_position, detect_orientation)
      
                robot_control.control_gripper(100)
                robot_control = RobotInit(0.3)
                robot_control.moveit_move(np.array([0.251, -0.154, 0.599]), detect_orientation)
                rospy.sleep(1)
    
    