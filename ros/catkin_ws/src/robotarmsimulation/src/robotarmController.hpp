#ifndef ROBOTARMCONTROLLER_H_
#define ROBOTARMCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <cstring>
#include <string>
#include <vector>

class RobotarmController
{
  public:
    RobotarmController();
    ~RobotarmController();
  private:
    ros::NodeHandle mNodeHandler;
    ros::Subscriber mRobotarmCommandSubscriber;
    ros::Publisher mRobotarmPublisher;

    void initializeCommunication();
    void initializeVales();

    // Robotarm position variables
    std::vector<std::string> mJointNames;
    std::vector<double> mJointPositions;

    void sendCurrentStateToVisualizer();

    void handleCommand(std::string aCommand);

    void robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand);
};

#endif