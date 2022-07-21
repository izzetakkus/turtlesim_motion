#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from asyncore import loop
from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
import numpy as np

x = 0
y = 0
z = 0
yaw = 0

x_min = 0.0
y_min = 0.0
x_max = 11.0
y_max = 11.0


def poseCallback(msg):
    global x, y, z, yaw
    x = msg.x
    y = msg.y
    yaw = msg.theta

def getDistance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)


def setDesiredOrientation(desired_angle_radians,velocity_publisher):
    global yaw

    relative_angle_radians = desired_angle_radians - yaw

    print("relative angle: ",relative_angle_radians)
    
    if relative_angle_radians < 0:
        clockwise = True
    else:
        clockwise = False

    print("clockwise: ",clockwise)


    # clockwise = True if relative_angle_radians < 0 else False

    rotate(abs(relative_angle_radians)/4.0, abs(relative_angle_radians), clockwise, velocity_publisher )


def move(speed, desired_distance, is_forward, velocity_publisher):
    #Declare a velocity message to publish
    
    vel_msg = Twist()

    if(is_forward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    t0 = rospy.Time.now().to_sec()

    distance_moved = 0.0
    
    loop_rate = rospy.Rate(10) #publish the velocity at 10 Hz (10 times per second)
    
    while(distance_moved < desired_distance):
        
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        distance_moved = speed * (t1 - t0)
        print("Current Distance: ",distance_moved)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

def rotate(angular_speed_radians, relative_angle_radians, clockwise, velocity_publisher):

    global yaw
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    angular_speed = abs(angular_speed_radians)

    if(clockwise):
        vel_msg.angular.z = -abs(angular_speed) 
    else:
        vel_msg.angular.z = abs(angular_speed)
    
    loop_rate = rospy.Rate(10)

    t0 = rospy.Time.now().to_sec()
    
    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(vel_msg)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_radians
        loop_rate.sleep()

        if current_angle_degree > relative_angle_radians:
            rospy.loginfo("Reached")
            break
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    

def go_to_goal(x_goal, y_goal, velocity_publisher): 

    global x
    global y,yaw

    loop_rate = rospy.Rate(10)
    vel_msg = Twist()

    while (True):
        K_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x)**2) + ((y_goal-y) ** 2)))
        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal - y, x_goal-x)
        angular_speed = (desired_angle_goal - yaw)*K_angular

        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed

        velocity_publisher.publish(vel_msg)

        loop_rate.sleep()


        print('x=', x, ', y =',y, ', distance to goal', distance)

        if(distance < 0.01):
            break

def gridClean():

    global x,y,x_min,x_max,y_min,y_max

    loop_rate = rospy.Rate(10)

    x_goal = 1
    y_goal = 1
    velocity_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(velocity_topic,Twist,queue_size=10)

    go_to_goal(x_goal,y_goal,velocity_publisher)
    loop_rate.sleep()
    setDesiredOrientation(np.deg2rad(0),velocity_publisher)
    loop_rate.sleep()

    move(2.0,9.0,True,velocity_publisher)
    loop_rate.sleep()
    rotate(np.deg2rad(10),np.deg2rad(90),False,velocity_publisher)
    loop_rate.sleep()
    move(2,9,True,velocity_publisher)

    rotate(np.deg2rad(10),np.deg2rad(90),False,velocity_publisher)
    loop_rate.sleep()
    move(2,1,True,velocity_publisher)
    rotate(np.deg2rad(10),np.deg2rad(90),False,velocity_publisher)
    loop_rate.sleep()
    move(2,9,True,velocity_publisher)


    rotate(np.deg2rad(30),np.deg2rad(90),True,velocity_publisher)
    loop_rate.sleep()
    move(2,1,True,velocity_publisher)
    rotate(np.deg2rad(30),np.deg2rad(90),True,velocity_publisher)
    loop_rate.sleep()
    move(2,9,True,velocity_publisher)


def spiralClean():


    velocity_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(velocity_topic,Twist,queue_size=10)


    vel_msg = Twist()
    constant_speed = 4
    rk = 0.5
    loop_rate = rospy.Rate(1)

    while(True):

        rk = rk + 0.5
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = constant_speed

        print("Linear Velocity: %.2f "%vel_msg.linear.x)
        print("\nAngular Velocity: %.2f "%vel_msg.angular.z)

        velocity_publisher.publish(vel_msg)
        rospy.spin()
        loop_rate.sleep()


        if x > 10.5 or y > 10.5:
            break

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)





if __name__ == '__main__' :
    try:
        rospy.init_node('turtlesim_motion',anonymous= True)
        #declare velocity publisher 

        velocity_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(velocity_topic,Twist,queue_size=10)

        position_topic = '/turtle1/pose'
        pose_subscriber = rospy.Subscriber(position_topic,Pose,poseCallback)
        loop_rate = rospy.Rate(10)

        # move(1.0, 4.0, 1, velocity_publisher)
        # rotate(10, 80, 1, velocity_publisher)
        # go_to_goal(1, 2, velocity_publisher)

        # setDesiredOrientation(np.deg2rad(120), velocity_publisher);
        # loop_rate = rospy.Rate(10)
        # loop_rate.sleep();
        # setDesiredOrientation(np.deg2rad(-60), velocity_publisher);
        # loop_rate.sleep();
        # setDesiredOrientation(np.deg2rad(0), velocity_publisher);

        # gridClean()

        spiralClean()
        loop_rate.sleep()

        time.sleep(2)

    except rospy.ROSInterruptException: 
        rospy.loginfo("node terminated.")