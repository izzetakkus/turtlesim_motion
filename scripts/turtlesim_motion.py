#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

x = 0
y = 0
z = 0
yaw = 0

x_min = 0.0
y_min = 0.0
x_max = 11.0
y_max = 11.0


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

    rotate(relative_angle_radians, relative_angle_radians, clockwise, velocity_publisher )


def poseCallback(msg):
    global x, y, z, yaw
    x = msg.x
    y = msg.y
    yaw = msg.theta

def move(speed, desired_distance, is_forward, velocity_publisher):
    #Declare a velocity message to publish
    
    velocity_message = Twist()
    global x, y
    x0 = x
    y0 = y

    if(is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) #publish the velocity at 10 Hz (10 times per second)

    while(True):
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved = abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2 )))
        print(distance_moved)

        if not (distance_moved < desired_distance):
            rospy.loginfo("reached")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def rotate(angular_speed_radians, relative_angle_radians, clockwise, velocity_publisher):

    global yaw
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    angular_speed = abs(angular_speed_radians)

    if(clockwise):
        velocity_message.angular.z = -abs(angular_speed) 
    else:
        velocity_message.angular.z = abs(angular_speed)
    
    loop_rate = rospy.Rate(10)

    t0 = rospy.Time.now().to_sec()
    
    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_radians
        loop_rate.sleep()

        if current_angle_degree > relative_angle_radians:
            rospy.loginfo("Reached")
            break
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    

def go_to_goal(x_goal, y_goal, velocity_publisher): 

    global x
    global y,yaw

    loop_rate = rospy.Rate(10)
    velocity_message = Twist()

    while (True):
        K_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x)**2) + ((y_goal-y) ** 2)))
        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal - y, x_goal-x)
        angular_speed = (desired_angle_goal - yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)

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
    setDesiredOrientation(0,velocity_publisher)
    loop_rate.sleep()

    move(2,9,True,velocity_publisher)
    loop_rate.sleep()
    rotate(10,90,False,velocity_publisher)
    loop_rate.sleep()
    move(2,9,True,velocity_publisher)


    rotate(10,90,False,velocity_publisher)
    loop_rate.sleep()
    move(2,1,True,velocity_publisher)
    rotate(10,90,False,velocity_publisher)
    loop_rate.sleep()
    move(2,9,True,velocity_publisher)


    rotate(30,90,True,velocity_publisher)
    loop_rate.sleep()
    move(2,1,True,velocity_publisher)
    rotate(30,90,True,velocity_publisher)
    loop_rate.sleep()
    move(2,9,True,velocity_publisher)

    distance = getDistance(x,y,x_max,y_max)


if __name__ == '__main__' :
    try:
        rospy.init_node('turtlesim_motion',anonymous= True)
        #declare velocity publisher 

        velocity_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(velocity_topic,Twist,queue_size=10)

        position_topic = '/turtle1/pose'
        pose_subscriber = rospy.Subscriber(position_topic,Pose,poseCallback)

        # move(1.0, 4.0, 1, velocity_publisher)
        # rotate(10, 80, 1, velocity_publisher)
        #go_to_goal(1, 2, velocity_publisher)
        gridClean()

        time.sleep(2)

    except rospy.ROSInterruptException: 
        rospy.loginfo("node terminated.")