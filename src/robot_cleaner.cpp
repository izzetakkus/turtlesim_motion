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
#include <math.h>

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;

void mySigintHandler(int);
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed,double angle, bool clockwise);
double degree2radian(double angle_in_degree);
void poseCallback(const turtlesim::PoseConstPtr & pose_message);
void setDesiredOrientation(double desired_angle_radians);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);


double x{0};
double y{0};
double yaw{0};

turtlesim::Pose turtlesim_pose;

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

    pose_subscriber = node.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::spinOnce();

    velocity_publisher = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Rate loop_rate(10);

    /*
    cout << "Enter the speed: ";
    cin >> speed;

    cout << "Enter the desired distance: ";
    cin >> distance;

    cout << "Select the direction" << endl;
    cout << "[0] -----> Backward\n[1]-----> Forward" << endl;
    cin >> isForward;

    move(speed, distance , isForward);

    */

    /*
    cout << "Enter the angular velocity (degree/sec): ";
    cin >> angular_speed;

    cout << "Enter the desired angle (degree): ";
    cin >> angle;

    cout << "Select the direction" << endl;
    cout << "[0] -----> Anti Clockwise\n[1]-----> Clockwise" << endl;
    cin >> clockwise;

    rotate(degree2radian(angular_speed), degree2radian(angle), clockwise);
    

    setDesiredOrientation(degree2radian(120));
    ros::Rate loop_rate(0.5);
    loop_rate.sleep();
    setDesiredOrientation(degree2radian(-60));
    loop_rate.sleep();
    setDesiredOrientation(degree2radian(0));

    */

    turtlesim::Pose goal_pose;

    goal_pose.x = 1;
    goal_pose.y = 1;
    goal_pose.theta = 0;

    moveGoal(goal_pose,0.01);

    loop_rate.sleep();
    
    
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


void poseCallback(const turtlesim::PoseConstPtr & pose_message){
    
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;

}



double degree2radian(double angle_in_degree){
    return angle_in_degree * M_PI/180.0;
}

void setDesiredOrientation(double desired_angle_radians){

    double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
    bool clockwise = (relative_angle_radians<0) ? true:false ;
    rotate(abs(relative_angle_radians)/4, abs(relative_angle_radians), clockwise);

}

double getDistance(double act_x, double act_y, double des_x, double des_y){
    return sqrt( pow((act_x - des_x),2) + pow((act_y - des_y),2) );
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){

    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(10);
    double Kp{1.5};
    double Ka{4};

    do{
        /**** Propotional Control ****/

        //Linear velocity
        vel_msg.linear.x = Kp*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

        //Angular velocity

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = Ka*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x));
        velocity_publisher.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    } while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y));

    cout << "Reached the desired point!" << endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

