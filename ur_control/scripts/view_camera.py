#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image as msg_Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import cv2
import numpy as np
import time
class Listener:
    def __init__(self, img_topic, msg_topic):
        self.img_topic = img_topic
        self.msg_topic = msg_topic
        self.bridge = CvBridge()
        self.img = None
        self.msg = False
        print("1 wait...", self.msg)
        self.img_sub = rospy.Subscriber(img_topic, msg_Image, self.imageDepthCallback)
        self.msg_sub = rospy.Subscriber(msg_topic, Bool, self.messageCallback)
        print("2 wait...", self.msg)

    def imageDepthCallback(self, data):
        try:
            ## converting ros image messages to opencv images
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = cv_image
            # cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            # cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            # cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            # scale_percent = 60       # percent of original size
            # width = int(cv_image.shape[1] * scale_percent / 100)
            # height = int(cv_image.shape[0] * scale_percent / 100)
            # dim = (width, height)
            # cv_image_resized = cv2.resize(cv_image_norm, dim, interpolation = cv2.INTER_CUBIC)
            # depth_image = cv_image_resized

            # pix = (data.width/2, data.height/2)
            # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            # sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Color image", cv_image)
        k = cv2.waitKey(10)

        # if k == 27: # Press ESC
        #     cv2.destroyAllWindows()    
        # elif k == ord('s'): # Press 's' to save the image
        #     ts = str(time.time())
        #     cv2.imwrite("/root/chessboard_images/image_"+ts+".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        #     print("image_"+ts+".jpg"+" saved.")

    def messageCallback(self, data):
        print("wait...")
        rospy.loginfo(rospy.get_caller_id()+" -- Chessboard: "+str(data.data))
        self.msg = data.data

        if self.msg:    
            ts = str(time.time())
            if not os.path.exists("/root/chessboard_images"):
                os.makedirs("/root/chessboard_images")
            # save the current frame when the topic "/gazebo/chessboard_ok" sends True
            cv2.imwrite("/root/chessboard_images/image_"+ts+".jpg", self.img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            print("image_"+ts+".jpg"+" saved.")
            rospy.signal_shutdown(">>> CURRENT image saved, shutting down the node ...")


if __name__ == '__main__':

    rospy.init_node("camera_image_processor")
    img_topic = '/camera/color/image_raw'
    msg_topic = '/gazebo/chessboard_ok'
    listener = Listener(img_topic, msg_topic)
    print(listener.msg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
