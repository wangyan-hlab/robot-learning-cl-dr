#!/usr/bin/python3
# Randomly place a model into the world
# Author:   Wang, Yan
# Date:     2022/11/25
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

rospy.init_node('spawn_gazebo_random')

class SpawnerModelRandom:
    def __init__(self):
        self.spawner = GazeboModels('ur3_gazebo')
        self.name = "aruco_marker_4" 

    def place_model_random(self):
        delta_x_range = [-0.05, 0.05]
        delta_y_range = [-0.09, 0.09]
        delta_z_range = [-0.02, 0.02]
        delta_ax_range = [-0.52, 0.52]
        delta_ay_range = [-0.52, 0.52] 
        delta_az_range = [-0.52, 0.52]
        ranges = [delta_x_range, delta_y_range, delta_z_range, 
                delta_ax_range, delta_ay_range, delta_az_range]
        delta = [np.interp(np.random.random(), [0, 1.0], range) for range in ranges]
        print("-- DELTA --\n", delta)
        objpose = np.array([0.27, 0.12, 0.8, 0, 0, 1.57]) + delta
        models = [Model(self.name, list(objpose), file_type='sdf', reference_frame="world")]
        self.spawner.load_models(models)

        return models

    def get_pose(self, models):
        for m in models:
            print("-- MODEL POSE --\n", m.pose)

    def execute(self):
        """ Main function to be run. """
        models = self.place_model_random()
        self.get_pose(models)

if __name__ == "__main__":
    topic = '/gazebo/chessboard_ok'
    cb_spawner = SpawnerModelRandom()
    cb_spawner.execute()
    pub = rospy.Publisher(topic, Bool, queue_size=10)
    rate = rospy.Rate(1)   # 10Hz
    
    while not rospy.is_shutdown():
        chessboard_placed = True
        pub.publish(chessboard_placed)
        rate.sleep()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
