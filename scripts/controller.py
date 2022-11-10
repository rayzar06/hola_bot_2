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

# Team ID:		    1634
# Author List:		Sriram M , Muhammed Shehin , Joel S Thomas , Sarang Tk
# Filename:		    controller.py
# Functions:        signal_handler, cleanup, task2_goals_Cb, aruco_feedback_Cb, distance, linear_vel_x, linear_vel_y, angular_vel, inverse_kinematics, move2goal
# Nodes:		    controller_node


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
from time import sleep
import numpy as np
from tf.transformations import euler_from_quaternion	# Convert angles
################## GLOBAL VARIABLES ######################

PI = 22/7

x_goals = []
y_goals = []
theta_goals = []

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

hola_x = 0
hola_y = 0
hola_theta = 0

rospy.init_node('controller_node')
rate = rospy.Rate(1000)
right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

def signal_handler(sig, frame):
      
    # NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    global right_wheel_pub, left_wheel_pub, front_wheel_pub
    front_force = Wrench()
    right_force = Wrench()
    left_force = Wrench()

    front_force.force.x = 0
    right_force.force.x = 0
    left_force.force.x = 0

    front_wheel_pub.publish(front_force)
    right_wheel_pub.publish(right_force)
    left_wheel_pub.publish(left_force)


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
    global hola_x, hola_y, hola_theta 
    hola_x = msg.x 
    hola_y = msg.y 
    hola_theta = msg.theta


def distance(x_d, y_d):
    global hola_x,hola_y
    return math.sqrt((x_d-hola_x)**2+(y_d-hola_y)**2)


def linear_vel_x(x_d, y_d, const=50):
    global hola_x,hola_x,hola_theta
    return const*math.cos((math.atan2(hola_y-y_d,x_d-hola_x)-hola_theta))


def linear_vel_y(x_d, y_d, const=50):
    global hola_x,hola_x,hola_theta
    return const*math.sin((math.atan2(hola_y-y_d,x_d-hola_x)-hola_theta))


def angular_vel(theta_d, constant=100):
    global hola_theta
    return constant * (theta_d - hola_theta)


def inverse_kinematics(v_x, v_y,v_w):
    global hola_x,hola_y,hola_theta
    const_matrix = np.array([[-0.17483,1,0],[-0.17483,-math.cos(PI/3),-math.sin(PI/3)],[-0.17483,-math.cos(PI/3),math.sin(PI/3)]])
    vel_matrix = np.array([[v_w],[v_x],[v_y]])
    wheel_vel = np.matmul(const_matrix,vel_matrix)
    return wheel_vel


def move2goal(x_d,y_d,theta_d):
    global hola_x,hola_y,hola_theta
    #y_d = 499-y_d
    vel_front = Wrench()
    vel_left = Wrench()
    vel_right = Wrench()
    del_theta = theta_d - hola_theta
    while distance(x_d,y_d) >= 0.1 :
        const =3*distance(x_d, y_d) 
        
        wheel_vel = inverse_kinematics(linear_vel_x(x_d,y_d,const),linear_vel_y(x_d,y_d,const),angular_vel(theta_d))
    
        vel_front.force.x = wheel_vel[0]
        vel_right.force.x = wheel_vel[1]
        vel_left.force.x = wheel_vel[2]

        front_wheel_pub.publish(vel_front)
        right_wheel_pub.publish(vel_right)
        left_wheel_pub.publish(vel_left)

        rate.sleep()
    cleanup()



def main():
    global x_goals,y_goals,theta_goals,hola_x,hola_y,hola_theta
    signal.signal(signal.SIGINT, signal_handler)
    rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
    rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)

    while not rospy.is_shutdown():
        if x_goals and y_goals and theta_goals:
            for x_d,y_d,theta_d in zip(x_goals,y_goals,theta_goals):
                print("x_d = ",x_d,"y_d = ",y_d,"theta_d = ",theta_d)
                move2goal(x_d,y_d,theta_d)
                print("Reached")
                sleep(4)
            
            x_goals.clear()
            y_goals.clear()
            theta_goals.clear()
            
    rospy.signal_shutdown("Test cases completed")       



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

