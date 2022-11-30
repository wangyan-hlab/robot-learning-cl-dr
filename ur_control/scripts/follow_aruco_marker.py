#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from ur_control import transformations, conversions
from ur_control.arm import Arm
import numpy as np

class PoseListener:
    def __init__(self, topic):
        self.pose_sub = rospy.Subscriber(topic, Pose, self.poseCallback)
        self.move_pub = rospy.Publisher('/robot/move_status', Bool, queue_size=10)
        self.rate = rospy.Rate(10) # Hz

    def poseCallback(self, data):
        marker_to_wrist3_mat = conversions.from_pose(data)
        print("[INFO] marker_to_wrist3_mat:", marker_to_wrist3_mat)

        global arm
        arm = Arm(
            ft_sensor=True,  # get Force/Torque data or not
            gripper=True,  # Enable gripper
        )
        cpose = arm.end_effector()
        wrist3_to_base_mat = transformations.quaternion_matrix(cpose[3:])
        wrist3_to_base_mat[:3, 3] = cpose[:3]
        marker_to_base_mat = np.dot(wrist3_to_base_mat, marker_to_wrist3_mat)
        target_mat = np.dot(marker_to_base_mat, transformations.euler_matrix(np.pi, 0, 0))
        target_pose = transformations.pose_quaternion_from_matrix(target_mat)
        deltaz = np.array([0.0, 0.0, -0.03, 0., 0., 0.])
        target_pose = transformations.pose_euler_to_quaternion(target_pose, deltaz, ee_rotation=True)
        rospy.loginfo("target_pose = {}".format(target_pose))
        # tell the marker detector to stop detecting because the robot is moving
        self.move_pub.publish(True)
        self.rate.sleep()
        arm.set_target_pose(pose=target_pose, wait=True, t=10.0)
        print("distance to target: ", np.linalg.norm(arm.end_effector()[:3] - target_pose[:3]))
        
        if np.linalg.norm(arm.end_effector()[:3] - target_pose[:3]) < 1e-3:
            rospy.loginfo("Target reached!")
            self.move_pub.publish(False)
            rospy.signal_shutdown("Quit")
        else:
            print("Target not reached!")

        return marker_to_wrist3_mat

if __name__ == '__main__':

    rospy.init_node("marker_pose_processor")
    topic = '/marker/mat_to_wrist3'
    listener = PoseListener(topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")