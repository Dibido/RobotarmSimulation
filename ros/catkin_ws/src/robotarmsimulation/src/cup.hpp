#ifndef CUP_HPP_
#define CUB_HPP_

#include <string>
#include <iostream>
#include <cstdio>
#include <ros/ros.h>


//
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
//

class Cup
{
public:
  Cup(std::string aTopic);
  ~Cup();
  void showCup();
  void handleCollision();

private:
  std::string topic;
  void publishCup();


  ros::NodeHandle n;
  ros::Publisher marker_pub;
};

#endif