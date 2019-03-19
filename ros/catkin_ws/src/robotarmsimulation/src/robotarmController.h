#ifndef ROBOTARMCONTROLLER_H_
#define ROBOTARMCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <cstring>
#include <string>

class RobotarmController
{
  public:
    RobotarmController();
    ~RobotarmController();
  private:
    ros::NodeHandle mNodeHandler;
    ros::Subscriber mRobotarmCommandSubscriber;
    ros::Publisher mRobotarmPublisher;

    // Robotarm position variables


    void robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand);
};

#endif