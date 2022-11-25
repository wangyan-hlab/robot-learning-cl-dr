#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+" -- I heard "+str(data.data))

def listener():
    rospy.init_node('chess_listener', anonymous=True)
    rospy.Subscriber("/gazebo/chessboard_ok", Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
