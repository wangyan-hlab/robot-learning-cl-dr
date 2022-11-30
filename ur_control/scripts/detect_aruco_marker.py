#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image as msg_Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from ur_control import transformations
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import sys
import argparse
import numpy as np
import yaml
import os
path = os.path.join(os.getcwd(), "ros_ws/src/glozzom/ur_control/")

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_4X4_50", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())
cam_conf_file = open(os.path.join(path, "config/realsense_camera_paras.yaml"), 'r')
cam_dict = yaml.load(cam_conf_file, Loader=yaml.Loader)
cam_intrinsic_mat = np.asarray(cam_dict['camera']['intrinsic_matrix'])
# print("[INFO] camera intrinsic matrix: {}".format(cam_intrinsic_mat))
cam_dist = np.asarray(cam_dict['camera']['dist'])
# print("[INFO] camera dist: {}".format(cam_dist))
lens_to_cambase_mat = np.asarray(cam_dict['camera']['lens_to_base_matrix'])
# print("[INFO] camera lens to baselink matrix: {}".format(lens_to_cambase_mat))
cambase_to_wrist3_mat = np.asarray(cam_dict['camera']['base_to_wrist3_matrix'])
# print("[INFO] camera baselink to robot wrist3 matrix: {}".format(cambase_to_wrist3_mat))

IMAGE_DEPTH = 0.0   # TODO: object depth in the image, tvec[2] might be replaced with this value

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    # "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    # "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    # "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    # "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.move_sub = rospy.Subscriber('/robot/move_status', Bool, self.robotMoveCallback)
        self.pose_pub = rospy.Publisher("/marker/mat_to_wrist3", Pose, queue_size=1)
        self.move_msg = False
        self.rate = rospy.Rate(10) # Hz

    def robotMoveCallback(self, data):
        self.move_msg = data.data
        if self.move_msg:
            rospy.loginfo("Robot is moving, stopping detecting markers!")
            rospy.signal_shutdown("Quit")

    def imageDepthCallback(self, data):
        try:
            ## initialize the video stream and allow the camera sensor to warm up
            # print("[INFO] starting video stream...")
            # # vs = VideoStream(src=0).start()
            # cap = cv2.VideoCapture(0)
            # time.sleep(2.0)
            # if not cap.isOpened():
            #     raise IOError("Cannot open webcam.")
            # ret, cv_image = cap.read()

            ## converting ros image messages to opencv images
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            frame = cv2.resize(cv_image, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # detect ArUco markers in the input frame
            
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0 and (not self.move_msg):
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # extract the marker corners (which are always returned
                    # in top-left, top-right, bottom-right, and bottom-left
                    # order)
                    # cv2.aruco.drawDetectedMarkers(frame, corners)
                    print("-- Marker Info --\n Corner:\n {}\n ID: {}".format(markerCorner, markerID))
                    (rvec, tvec) = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, cam_intrinsic_mat, cam_dist)
                    # print("rvec: {}".format(rvec))
                    # print("tvec: {}".format(tvec))
                    # Draw Axis
                    # cv2.drawFrameAxes(frame, cam_intrinsic_mat, cam_dist, rvec, tvec, 0.01)
                    (rotmat, _) = cv2.Rodrigues(rvec)
                    tvec = tvec.reshape((3,))
                    # print("rotmat: {}".format(rotmat))
                    # print("tvec: {}".format(tvec))
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                    # compute and draw the center (x, y)-coordinates of the
                    # ArUco marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    if markerID == 1:
                        marker_to_lens_mat = np.identity(4)
                        marker_to_lens_mat[:3, 3] = tvec
                        marker_to_lens_mat[:3, :3] = rotmat
                        # print("CameraLensFrame/Marker_1 transformation: {}".format(marker_to_lens_mat))
                        marker_to_cambase_mat = np.dot(lens_to_cambase_mat, marker_to_lens_mat)
                        # print("CameraBaseFrame/Marker_1 transformation: {}".format(marker_to_cambase_mat))
                        marker_to_wrist3_mat = np.dot(cambase_to_wrist3_mat, marker_to_cambase_mat)
                        # print("RobotWrist3Frame/Marker_1 transformation: {}".format(marker_to_wrist3_mat))
                        # publish marker_to_wrist3_pose to the topic "/marker/mat_to_wrist3"
                        marker_to_wrist3_pose = transformations.pose_quaternion_from_matrix(marker_to_wrist3_mat)
                        rospy.loginfo("marker_to_wrist3_pose = {}".format(marker_to_wrist3_pose))

                        p = Pose()
                        p.position.x = marker_to_wrist3_pose[0]
                        p.position.y = marker_to_wrist3_pose[1]
                        p.position.z = marker_to_wrist3_pose[2]
                        p.orientation.x = marker_to_wrist3_pose[3]
                        p.orientation.y = marker_to_wrist3_pose[4]
                        p.orientation.z = marker_to_wrist3_pose[5]
                        p.orientation.w = marker_to_wrist3_pose[6]
                        self.pose_pub.publish(p)
                        self.rate.sleep()

                    elif markerID == 2:
                        print("ImageFrame/Marker 2 at: ({}, {})".format(cX, cY))
                    elif markerID == 3:
                        print("ImageFrame/Marker 3 at: ({}, {})".format(cX, cY))
                    else:
                        print("Marker 1, 2, or 3 not found in view!")
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the frame
                    cv2.putText(frame, str(markerID),
                                (topLeft[0], topLeft[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)

        except CvBridgeError as e:
            print(e)
            return
        
        cv2.imshow("Color image", frame)
        cv2.waitKey(1)

if __name__ == '__main__':

    # verify that the supplied ArUCo tag exists and is supported by
    # OpenCV
    if ARUCO_DICT.get(args["type"], None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
            args["type"]))
        sys.exit(0)
    # load the ArUCo dictionary and grab the ArUCo parameters
    print("[INFO] detecting '{}' tags...".format(args["type"]))
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters_create()

    # initialize rosnode to process camera image message
    rospy.init_node("depth_image_processor")
    topic = '/camera/color/image_raw'
    listener = ImageListener(topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
