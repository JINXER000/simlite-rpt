#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <std_srvs/Empty.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

double x = 8.17;
double y = 0.11;
ros::Publisher pos_pub;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  x+=msg->linear.x*0.025;
  y+=msg->angular.z*0.025;
  gazebo_msgs::ModelState msgMS;
  msgMS.model_name = "unit_cylinder_15";
  msgMS.pose.position.x = x;
  msgMS.pose.position.y = y;
  msgMS.pose.position.z = 0.5;
  pos_pub.publish(msgMS);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "refPub");
    ros::NodeHandle n;

    pos_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
    ros::Subscriber cmd_sub = n.subscribe("/turtle1/cmd_vel",1,cmdCallback);
    ROS_WARN("cfg ok!\n");
    ros::spin();

    return 0;
}
