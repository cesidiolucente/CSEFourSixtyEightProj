#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from copy import copy
import math
import rviz
import random
global pose




def callback(msg):
    callback.counter+=1 #makes sure that the code doesnt run more often then necessary, reducing weightn
    if(callback.counter%5==0):
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        position=msg.pose.pose.orientation.z
        #rospy.loginfo(position)




    return
callback.counter=0

def wallfollow(Float64):

        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        slope= Float64




        #rospy.loginfo(slope.data)

        if(slope.data==999): #if theres a division error in perception then slope is 999 this checks for that

            return
        x=-.1
        y=.1
        if(slope.data<x): #if robot is pointing away from wall, turn towards wall
            #rospy.loginfo('trigger1')
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = .5

            velocity_publisher.publish(vel_msg)
        elif(slope.data>y): #if robot is pointing towards the wall turn away a little
            #rospy.loginfo('trigger2')
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = -.5

            velocity_publisher.publish(vel_msg)
        else: #if parrallel to wall move forwards
            vel_msg.linear.x = 1
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            velocity_publisher.publish(vel_msg)


            return


if __name__ == '__main__':

    rospy.init_node('bug2', anonymous=True)


    rospy.Subscriber("/odom", Odometry, callback)
    rospy.Subscriber('slope', Float64 , wallfollow)


    rospy.spin()
