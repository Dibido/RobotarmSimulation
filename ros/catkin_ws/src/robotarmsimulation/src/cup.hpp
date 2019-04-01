#ifndef CUP_HPP_
#define CUB_HPP_

#include <string>
#include <iostream>
#include <cstdio>
#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


#define CUP_SIZE 0.04
#define GRIPPER_DEPTH 0.01

namespace COLORS
{
enum ColorState
{
  RED,
  GREEN
};

};

class Cup
{
public:
  Cup(std::string aTopic, float aOriginalX = 0.4, float aOriginalY = 0.0, float aOriginalZ = 0.0);
  ~Cup();
  void showCup(COLORS::ColorState color);
  void handleCollision();

private:
  std::string topic;
  void publishCup();

  void setColor(COLORS::ColorState color, visualization_msgs::Marker& marker);
  bool isOpbjectInGripper(tf::StampedTransform& object);
  float calculateFallingTime(tf::StampedTransform& object);


  float cupPosY;
  float cupPosX;
  float cupPosZ;
  COLORS::ColorState color = COLORS::RED;
  ros::Time timeFrameTime;

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  tf::TransformListener listener;
};

#endif