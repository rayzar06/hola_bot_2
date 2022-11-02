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
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################


from cmath import cos
import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	    # Message type used for receiving goals
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import time
import math		# If you find it useful

import numpy as np
from tf.transformations import euler_from_quaternion	# Convert angles
################## GLOBAL VARIABLES ######################

PI = 22/7

x_goals = [50]
y_goals = [350]
theta_goals = [0]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

hola_x = 0
hola_y = 0
hola_theta = 0

init_x = 0
init_y = 0
init_theta = 0

real_x = 0
real_y = 0



def signal_handler(sig, frame):
      
    # NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    global right_wheel_pub, left_wheel_pub, front_wheel_pub
    rospy.signal_shutdown("Test cases completed")
    pass


def task2_goals_Cb(msg):
    global x_goals, y_goals, theta_goals
    x_goals.clear()
    y_goals.clear()
    theta_goals.clear()

    for waypoint_pose in msg.poses:
        x_goals.append(waypoint_pose.position.x)
        y_goals.append(waypoint_pose.position.y)

        orientation_q = waypoint_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        theta_goal = euler_from_quaternion (orientation_list)[2]
        theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
    global hola_x, hola_y, hola_theta , init_x, init_y, init_theta , real_x, real_y
    if init_x == 0 and init_y == 0 and init_theta == 0:
        init_x = msg.x
        init_y = msg.y 
        init_theta = msg.theta


    hola_x = msg.x 
    hola_y = msg.y 
    hola_theta = msg.theta


def linear_vel_x(x_d, y_d, const):
    return const*math.cos(math.atan2(y_d-hola_y,x_d-hola_x)-hola_theta)


def linear_vel_y(x_d, y_d, const):
    return const*math.sin(math.atan2(y_d-hola_y,x_d-hola_x)-hola_theta)


def inverse_kinematics(x_d,y_d,theta_d):
    global nhola_x,nhola_y,nhola_theta

    right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
    front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
    left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)
    rate = rospy.Rate(100)
    const_matrix = np.array([[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.67,0,0.33]])
    vel_matrix  = np.array([linear_vel_x(x_d,y_d,1),linear_vel_y(x_d,y_d,1),10*theta_d])
    ang_vel_matrix = np.matmul(const_matrix,vel_matrix)
    while not rospy.is_shutdown():
        del_x = x_d - hola_x
        del_y = y_d - hola_y
        del_theta = theta_d - hola_theta
        vel_front = Wrench()
        vel_left = Wrench()
        vel_right = Wrench()

        front = ang_vel_matrix[0]
        right = ang_vel_matrix[1]
        left = ang_vel_matrix[2]
        
        vel_front.force.x =500* front * math.cos(-PI/6)
        vel_front.force.y =500* front * math.sin(-PI/6)
        vel_front.force.z = 0

        vel_left.force.x =500* left * math.cos(-PI/6)
        vel_left.force.y =500* left * math.sin(PI/6)
        vel_left.force.z = 0

        vel_right.force.x =500* right * math.cos(-PI/6)
        vel_right.force.y =500* right * math.sin(-PI/6)
        vel_right.force.z = 0

        front_wheel_pub.publish(vel_front)
        left_wheel_pub.publish(vel_left)
        right_wheel_pub.publish(vel_right)
        if del_x == 0 and del_y == 0:
            vel_front.force.x = 0
            vel_front.force.y = 0

            vel_left.force.x = 0
            vel_left.force.y = 0

            vel_right.force.x = 0
            vel_right.force.y = 0
            front_wheel_pub.publish(vel_front)
            left_wheel_pub.publish(vel_left)
            right_wheel_pub.publish(vel_right)
            break
        rate.sleep()




def main():
    global x_goals,y_goals,theta_goals,hola_x,hola_y,hola_theta
    rospy.init_node('controller_node')

    signal.signal(signal.SIGINT, signal_handler)

    rate = rospy.Rate(100)
    rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
    rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)

    while not rospy.is_shutdown():
        if x_goals and y_goals and theta_goals:
            for x_d,y_d,theta_d in zip(x_goals,y_goals,theta_goals):
                print("x_d = ",x_d,"y_d = ",y_d,"theta_d = ",theta_d)
                inverse_kinematics(x_d,y_d,theta_d)
            '''   
            x_goals.clear()
            y_goals.clear()
            theta_goals.clear()'''
        rate.sleep()
    rospy.signal_shutdown("Test cases completed")       

    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

