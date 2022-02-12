#! /home/znfs/pyenv_pyrobot_python2/bin/python

from pyrobot import Robot
import numpy as np
import time
import math
from pyrobot.utils.util import MoveitObjectHandler
from robot_utils import *

robot = Robot('sawyer', use_base=False, use_camera=False)


def joint_position_control():
    robot.arm.go_home()
    target_joint = [0.704, -0.455, -0.159, 1.395, -1.240, 1.069, 2.477]
    robot.arm.set_joint_positions(target_joint, plan=False)
    robot.arm.go_home()


def joint_velocity_control():
    def sin_wave(t, f, A):
        return A * math.cos(2 * math.pi * f * t)

    A = 0.2
    f = 0.4
    robot.arm.go_home()
    start_time = time.time()
    robot.arm.move_to_neutral()
    while time.time() - start_time < 35:
        elapsed_time = time.time() - start_time
        vels = [sin_wave(elapsed_time, f, A)] * robot.arm.arm_dof
        robot.arm.set_joint_velocities(vels)
        time.sleep(0.01)


def joint_torque_control():
    def spring_damping(position_err, velocity_err, spring_coef, damping_coef):
        torques = -np.multiply(spring_coef, position_err)
        torques -= np.multiply(damping_coef, velocity_err)
        return torques

    ini_joint_angles = np.array(robot.arm.get_joint_angles())

    spring_coef = np.array([30.0, 45.0, 15.0, 15.0, 9.0, 6.0, 4.5])
    damping_coef = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    robot.arm.move_to_neutral()
    while True:
        joint_angles = np.array(robot.arm.get_joint_angles())
        joint_velocities = np.array(robot.arm.get_joint_velocities())
        pos_err = joint_angles - ini_joint_angles
        vel_err = joint_velocities
        joint_torques = spring_damping(pos_err, vel_err, spring_coef, damping_coef)
        robot.arm.set_joint_torques(joint_torques)
        time.sleep(0.001)


def ee_pose_control():
    target_poses = [{'position': np.array([0.8219, 0.0239, 0.0996]),
                     'orientation': np.array([[-0.3656171, 0.6683861, 0.6477531],
                                              [0.9298826, 0.2319989, 0.2854731],
                                              [0.0405283, 0.7067082, -0.7063434]])},
                    {'position': np.array([0.7320, 0.1548, 0.0768]),
                     'orientation': np.array([0.1817, 0.9046, -0.1997, 0.3298])},
                    ]
    robot.arm.go_home()
    time.sleep(1)
    for pose in target_poses:
        robot.arm.set_ee_pose(plan=True, **pose)
        time.sleep(1)


def ee_cartesian_path_control():
    plan = False
    robot.arm.move_to_neutral()
    time.sleep(1)
    displacement = np.array([0.15, 0, 0])
    robot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    displacement = np.array([0., 0.15, 0])
    robot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    displacement = np.array([0., 0., 0.15])
    robot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    robot.arm.go_home()


def moveit_planning_with_obstacle():
    obstacle_handler = MoveitObjectHandler()
    # Add a table
    # position and orientation (quaternion: x, y, z, w) of the table
    pose = [0.8, 0.0, -0.23, 0., 0., 0., 1.]
    # size of the table (x, y, z)
    size = (1.35, 2.0, 0.1)
    obstacle_handler.add_table(pose, size)
    target_poses = [{'position': np.array([0.8219, 0.0239, -0.1]),
                     'orientation': np.array([[-0.3656171, 0.6683861, 0.6477531],
                                              [0.9298826, 0.2319989, 0.2854731],
                                              [0.0405283, 0.7067082, -0.7063434]])},
                    {'position': np.array([0.7320, 0.1548, -0.15]),
                     'orientation': np.array([0.1817, 0.9046, -0.1997, 0.3298])},
                    ]
    robot.arm.go_home()
    time.sleep(1)
    for pose in target_poses:
        robot.arm.set_ee_pose(plan=True, **pose)
        time.sleep(1)
    robot.arm.go_home()


def main():
    # joint_position_control()
    # joint_velocity_control()
    # joint_torque_control()
    # ee_pose_control()
    # ee_cartesian_path_control()
    # moveit_planning_with_obstacle()
    robot_init = RobotInit(0.5)
    position = [0.6917, 0.1333, 0.3626]
    robot_init.moveit_move(position)


if __name__ == "__main__":
    main()
