#!/usr/bin/python3
# Description:  Randomly place a model into the world
# Author:       Wang, Yan
# Date:         2022/11/25
import argparse
import rospy
from std_msgs.msg import Bool
import numpy as np
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

# import copy
# from ur_control import transformations
# from ur_control.arm import Arm

from ur_gazebo.gazebo_spawner import GazeboModels
from ur_gazebo.model import Model
from ur_gazebo.basic_models import SPHERE, PEG_BOARD, BOX, SPHERE_COLLISION, get_box_model, get_button_model, get_peg_board_model

class SpawnerModelRandom:
    def __init__(self, pub_topic, sub_topic):
        self.spawner = GazeboModels('ur3_gazebo')
        self.name = "aruco_marker_4" 
        self.ranges = [[-0.05, 0.05], 
                       [-0.09, 0.09], 
                       [-0.02, 0.02], 
                       [-0.52, 0.52], 
                       [-0.52, 0.52], 
                       [-0.52, 0.52]]
        self.cb_pub = rospy.Publisher(pub_topic, Bool, queue_size=10)
        self.cb_sub = rospy.Subscriber(sub_topic, Bool, 
                                       callback=self.reset_model)
        self.rate = rospy.Rate(1) # Hz

    def place_model_random(self):
        delta = [np.interp(np.random.random(), [0, 1.0], range) for range in self.ranges]
        print("-- DELTA --\n", delta)
        objpose = np.array([0.27, 0.12, 0.8, 0, 0, 1.57]) + delta
        models = [Model(self.name, list(objpose), file_type='sdf', reference_frame="world")]
        self.spawner.load_models(models)
        self.cb_pub.publish(True)

    def get_pose(self):
        for m in self.models:
            print("-- MODEL POSE --\n", m.pose)

    def reset_model(self, imgsave):
        if imgsave:
            delta = [np.interp(np.random.random(), [0, 1.0], range) for range in self.ranges]
            print("-- DELTA --\n", delta)
            objpose = np.array([0.27, 0.12, 0.8, 0, 0, 1.57]) + delta
            models = [Model(self.name, list(objpose), file_type='sdf', reference_frame="world")]
            self.spawner.reset_models(models)
            self.cb_pub.publish(True)
            self.rate.sleep()
        else:
            print('>>> WAIT the image to be saved')

if __name__ == "__main__":

    rospy.init_node('spawn_gazebo_random')
    pub_topic = '/gazebo/chessboard_ok'
    sub_topic = '/opencv/image_save'
    
    cb_spawner = SpawnerModelRandom(pub_topic, sub_topic)
    cb_spawner.place_model_random() 
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
