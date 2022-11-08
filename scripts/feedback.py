#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		1634
# Author List:		Sriram M
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


######################## IMPORT MODULES ##########################

import numpy				              # If you find it required
import rospy 				
from sensor_msgs.msg import Image 	      # Image is the message type for images in ROS
from cv_bridge import CvBridge	          # Package to convert between ROS and OpenCV Images
import cv2	
import cv2.aruco as aruco		                  # OpenCV Library
import math				                  # If you find it required
from geometry_msgs.msg import Pose2D
# Required to publish ARUCO's detected position & orientation 
############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D,queue_size=40)
aruco_msg = Pose2D()
pose = Pose2D()
##################### FUNCTION DEFINITIONS #######################



def findArucoMarkers(img, markerSize = 4, totalMarkers=250, draw=True):
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(img, arucoDict, parameters = arucoParam)
    if draw:
        aruco.drawDetectedMarkers(img, corners, ids) 
    pose_estimation(corners)
    cv2.imshow("frame", img)
    cv2.waitKey(1)
    



def pose_estimation(corners):
    global pose
    pose.x = int(corners[0][0][0][0] + (corners[0][0][1][0] - corners[0][0][0][0])/2)
    pose.y = int(corners[0][0][0][1] + (corners[0][0][2][1] - corners[0][0][0][1])/2)
    pose.theta = math.atan2(-corners[0][0][1][1] + corners[0][0][0][1] , corners[0][0][1][0] - corners[0][0][0][0])
    pose.y = 499 -pose.y
    aruco_publisher.publish(pose)
    print(pose.x, pose.y, pose.theta)
    

def callback(data):
# Bridge is Used to Convert ROS Image message to OpenCV image
    br = CvBridge()
    rospy.loginfo("receiving camera frame")
    get_frame = br.imgmsg_to_cv2(data, "mono8")		                                   # Receiving raw image in a "grayscale" format
    current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

    findArucoMarkers(current_frame)

      
def main():
    rospy.init_node('aruco_feedback_node')  
    rospy.Subscriber('overhead_cam/image_raw', Image, callback,queue_size=20)
    rospy.spin()
  
if __name__ == '__main__':
    main()
