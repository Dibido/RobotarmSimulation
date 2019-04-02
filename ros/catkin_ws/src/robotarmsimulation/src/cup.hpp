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
  /**
   * @brief Construct a new Cup object
   * 
   * @param aTopic - The name of the topic this object will publish itself
   * @param aOriginalX - X location of the start position
   * @param aOriginalY - Y location of the start position
   * @param aOriginalZ - Z location of the start position
   */
  Cup(std::string aTopic, float aOriginalX = 0.4, float aOriginalY = 0.0, float aOriginalZ = 0.0);
  ~Cup();

  /**
   * @brief This function will detect collision and act appropriately
   * 
   */
  void handleCollision();

private:
  /**
   * @brief This function will create a frame where the cup object can be shown
   * 
   */
  void publishCup() const;

  /**
   * @brief 
   * 
   * @param color 
   */
  void showCup(const COLORS::ColorState color) const;

  /**
   * @brief This function will draw a sphere
   * 
   * @param color - The color u want ur sphere to be
   * @param frameName - The frame name u would like the sphere to be drawn
   */
  void showMarker(const COLORS::ColorState color,const std::string &frameName,const float offset) const;

  /**
   * @brief Set the Color of the cup
   * 
   * @param color - The color u would like ur object to be
   * @param marker - A reference of the object u want to color
   */
  void setColor(const COLORS::ColorState color, visualization_msgs::Marker& marker) const;

  /**
   * @brief This function will check if the object is touching the gripper
   * 
   * @param object - The dictence from the cup relitive from the gripper
   * @return true - Gripper is touching the object
   * @return false - Gripper is not touching the object
   */
  bool isOpbjectInGripper(const tf::StampedTransform& object) const;

  /**
   * @brief This function will calculate the falling speed of the object
   * 
   * @param object - The location of the object reletive to the world
   * @return float - The falling time of the object in seconds before it hits the ground
   */
  float calculateFallingTime(const tf::StampedTransform& object) const;

  /**
   * @brief Topic name of the object
   * 
   */
  std::string topic;

  /**
   * @brief The x,y and z positions of the cup
   * 
   */
  float cupPosY;
  float cupPosX;
  float cupPosZ;

  /**
   * @brief The set color ur cup wil be displayed
   * 
   */
  COLORS::ColorState color = COLORS::RED;

  /**
   * @brief The time when the last frame was displayed. This valiable is used to know how mutch time is passed between the current frame and the past frame
   * 
   */
  ros::Time pastFrameTime;

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  tf::TransformListener listener;
};

#endif