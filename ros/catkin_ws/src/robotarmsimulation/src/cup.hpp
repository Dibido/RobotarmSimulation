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

//Frames
#define FRAME_NAME_GRIPPER_LEFT "/gripper_left"
#define FRAME_NAME_GRIPPER_RIGHT "/gripper_right"
#define WORLD_OBJECT "/base_link"


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
  Cup(std::string aTopic);
  ~Cup();
  void handleCollision();

private:
  std::string topic;
  void publishCup();

  void setColor(COLORS::ColorState color, visualization_msgs::Marker& marker);
  bool isOpbjectInGripper(tf::StampedTransform& object);
  float calculateFallingTime(tf::StampedTransform& object);

  void showCup(COLORS::ColorState color);
  void showMarker(COLORS::ColorState color,std::string frameName);

  float cupPosY = 0;
  float cupPosX = 0.4;
  float cupPosZ = 0;
  COLORS::ColorState color = COLORS::RED;
  ros::Time timeFrameTime;

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  tf::TransformListener listener;
};

#endif