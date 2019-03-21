#ifndef CUP_HPP_
#define CUB_HPP_

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Cup
{
public:
  Cup(std::string aTopic);
  ~Cup();

private:
  std::string topic;
  void publishCup();


  ros::NodeHandle n;
  ros::Publisher marker_pub;
};

#endif