/*
 * robot_cleaner.cpp
 *
 *  Created on: Jul 5, 2022
 *      Author: izzet
 */

#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

ros::Publisher velocity_publisher;

void mySigintHandler(int);
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed,double angle, bool clockwise);
double degree2radian(double angle_in_degree);
void poseCallback(turtlesim::Pose msg);

double x{0};
double y{0};
double yaw{0};

using namespace std;
 
int main(int argc, char  **argv )

{
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle node;
    signal(SIGINT, mySigintHandler);

    double speed{0.0},angular_speed{0.0};
    double angle{0.0};
    double distance{0.0};
    bool isForward{true},clockwise{true};

    ros::Subscriber sub = node.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::spinOnce();

    velocity_publisher = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    // cout << "Enter the speed: ";
    // cin >> speed;

    // cout << "Enter the desired distance: ";
    // cin >> distance;

    // cout << "Select the direction" << endl;
    // cout << "[0] -----> Backward\n[1]-----> Forward" << endl;
    // cin >> isForward;

    // move(speed, distance , isForward);


    cout << "Enter the angular velocity (degree/sec): ";
    cin >> angular_speed;

    cout << "Enter the desired angle (degree): ";
    cin >> angle;

    cout << "Select the direction" << endl;
    cout << "[0] -----> Anti Clockwise\n[1]-----> Clockwise" << endl;
    cin >> clockwise;

    rotate(degree2radian(angular_speed), degree2radian(angle), clockwise);

    return 0;
}


void mySigintHandler(int sig)
{
  ROS_INFO("Node terminated !");
  ros::shutdown();
}

void move(double speed, double distance, bool isForward){

    geometry_msgs::Twist vel_msg;


    if(isForward){
        vel_msg.linear.x = abs(speed);
    }
    else{
        vel_msg.linear.x = -abs(speed);
    }

    vel_msg.linear.y, vel_msg.linear.z = 0;

    vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();

    double current_distance = 0.0;

    ros::Rate loop_rate(100); // 100 Hz

    do{
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        cout << "Current distance: " << current_distance << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);

}

void rotate(double angular_speed,double angle, bool clockwise){
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    if(clockwise)
        vel_msg.angular.z = -abs(angular_speed);
    else
        vel_msg.angular.z = abs(angular_speed);

    double current_angle{0.0};
    double t0 = ros::Time::now().toSec(); 
    ros::Rate loop_rate(100);

    do{
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec(); 
        current_angle = angular_speed*(t1-t0);
        ros::spinOnce();
        loop_rate.sleep();

    } while(current_angle < angle);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}


void poseCallback(turtlesim::Pose msg){
   x = msg.x;
   y = msg.y;
   yaw = msg.theta;
}


double degree2radian(double angle_in_degree){
    return angle_in_degree * (M_PI/180.0);
}
