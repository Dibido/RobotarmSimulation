#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotarmClient");
  // Create robotarmController publisher
  ros::NodeHandle lNodeHandler;
  ros::Publisher lRobotarmCommandPublisher = lNodeHandler.advertise<std_msgs::String>("robotarmCommand", 1000);
  // Publish commands
  if(ros::ok())
  {
    sleep(1);
    std_msgs::String msg;
    // msg.data = "#0P2000#1P1700#2P900#3P1100#4P1500#5P1500T3000\n";
    // ROS_INFO("sending command : %s", msg.data.c_str());
    // lRobotarmCommandPublisher.publish(msg);
    // ros::spinOnce();
    // sleep(5);
    msg.data = "#0P1500#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1200#2P1350#3P1600#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1200#2P1350#3P1600#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(4);
    msg.data = "#0P1500#1P1200#2P1150#3P1400#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(4);
    msg.data = "#0P1500#1P1200#2P1150#3P1400#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(4);
    msg.data = "#0P1500#1P1200#2P1150#3P1400#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(4);
    msg.data = "#0P1500#1P1500#2P1500#3P1500#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
  }
  return 0;
}