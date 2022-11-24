#! /usr/bin/env python

# The MIT License (MIT)
#
# Copyright (c) 2018-2022 Cristian C Beltran-Hernandez
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Cristian C Beltran-Hernandez

import numpy as np
from ur_control import transformations, conversions
from ur_control.arm import Arm
import argparse
import random
import rospy
import timeit

# If Docker version of Python3 version is used in melodic,
# Install tf for python3 like this: pip install --extra-index-url https://rospypi.github.io/simple/ tf2_ros
# Then enable the following  lines to re-direct tf to the new library
import sys
sys.path[:0] = ['/usr/local/lib/python3.6/dist-packages/']
import tf

np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)

def get_jnt():
    rospy.loginfo(f"jnt_angles (rad) = {arm.joint_angles()}")
    rospy.loginfo(f"jnt_angles (deg) = {np.rad2deg(arm.joint_angles())}")

def get_pose():
    rospy.loginfo(f"cpose = {arm.end_effector()}")


def move_joints(wait=True):
    # desired joint configuration 'q'
    # q = [0, 0, 0, 0, 0, 0]
    q = [1.57, -1.57, 1.05, -1.57, -1.57, 0]
    # go to desired joint configuration
    # in t time (seconds)
    # wait is for waiting to finish the motion before executing
    # anything else or ignore and continue with whatever is next
    arm.set_joint_positions(position=q, wait=wait, t=5.0)

def move_x(delta, wait=True):
    cpose = arm.end_effector()
    deltax = np.array([delta, 0., 0., 0., 0., 0.])
    cpose = transformations.pose_euler_to_quaternion(cpose, deltax, ee_rotation=True)
    arm.set_target_pose(pose=cpose, wait=wait, t=1.0)

def move_y(delta, wait=True):
    cpose = arm.end_effector()
    deltay = np.array([0., delta, 0., 0., 0., 0.])
    cpose = transformations.pose_euler_to_quaternion(cpose, deltay, ee_rotation=True)
    arm.set_target_pose(pose=cpose, wait=wait, t=1.0)

def move_z(delta, wait=True):
    cpose = arm.end_effector()
    deltaz = np.array([0., 0., delta, 0., 0., 0.])
    cpose = transformations.pose_euler_to_quaternion(cpose, deltaz, ee_rotation=True)
    arm.set_target_pose(pose=cpose, wait=wait, t=1.0)

def follow_trajectory():
    traj = [
        [2.4463, -1.8762, -1.6757, 0.3268, 2.2378, 3.1960],
        [2.5501, -1.9786, -1.5293, 0.2887, 2.1344, 3.2062],
        [2.5501, -1.9262, -1.3617, 0.0687, 2.1344, 3.2062],
        [2.4463, -1.8162, -1.5093, 0.1004, 2.2378, 3.1960],
        [2.3168, -1.7349, -1.6096, 0.1090, 2.3669, 3.1805],
        [2.3168, -1.7997, -1.7772, 0.3415, 2.3669, 3.1805],
        [2.3168, -1.9113, -1.8998, 0.5756, 2.3669, 3.1805],
        [2.4463, -1.9799, -1.7954, 0.5502, 2.2378, 3.1960],
        [2.5501, -2.0719, -1.6474, 0.5000, 2.1344, 3.2062],
    ]
    for t in traj:
        arm.set_joint_positions(position=t, wait=True, t=1.0)


def move_endeffector(wait=True):
    # get current position of the end effector
    cpose = arm.end_effector()
    # define the desired translation/rotation
    deltax = np.array([0.0, 0.02, 0.0, -0., 0., 0.])
    # add translation/rotation to current position
    cpose = transformations.pose_euler_to_quaternion(cpose, deltax, ee_rotation=True)
    # execute desired new pose
    # may fail if IK solution is not found
    arm.set_target_pose(pose=cpose, wait=True, t=1.0)


def get_random_valid_direction(plane):
    if plane == "XZ":
        return random.choice(["+X", "-X", "+Z", "-Z"])
    elif plane == "YZ":
        return random.choice(["+Y", "-Y", "+Z", "-Z"])
    elif plane == "XY":
        return random.choice(["+X", "-X", "+Y", "-Y"])
    else:
        raise ValueError("Invalid value for plane: %s" % plane)


def main():
    """ Main function to be run. """
    parser = argparse.ArgumentParser(description='Test force control')
    parser.add_argument('-j', '--jnt', action='store_true',
                        help='read current joint angles')
    parser.add_argument('-p', '--eef', action='store_true',
                        help='read current eef pose')
    parser.add_argument('-m', '--move', action='store_true',
                        help='move to joint configuration')
    parser.add_argument('-t', '--move_traj', action='store_true',
                        help='move following a trajectory of joint configurations')
    parser.add_argument('-e', '--move_ee', action='store_true',
                        help='move to a desired end-effector position')
    parser.add_argument('-g', '--gripper', action='store_true',
                        help='enable gripper')
    parser.add_argument('-mx', '--move_x',
                        nargs='?', const=0.04, type=float,
                        help='move the eef along its x-axis')
    parser.add_argument('-my', '--move_y',
                        nargs='?', const=0.04, type=float,
                        help='move the eef along its y-axis')
    parser.add_argument('-mz', '--move_z',
                        nargs='?', const=0.04, type=float,
                        help='move the eef along its z-axis')
                        

    args = parser.parse_args()

    rospy.init_node('ur3e_script_control')

    global arm
    arm = Arm(
        ft_sensor=True,  # get Force/Torque data or not
        gripper=args.gripper,  # Enable gripper
    )

    real_start_time = timeit.default_timer()
    ros_start_time = rospy.get_time()

    if args.jnt:
        get_jnt()
    if args.eef:
        get_pose()
    if args.move:
        move_joints()
    if args.move_traj:
        follow_trajectory()
    if args.move_ee:
        move_endeffector()
    if args.move_x:
        move_x(args.move_x)
    if args.move_y:
        move_y(args.move_y)
    if args.move_z:
        move_z(args.move_z)

    print("real time", round(timeit.default_timer() - real_start_time, 3))
    print("ros time", round(rospy.get_time() - ros_start_time, 3))


if __name__ == "__main__":
    main()
