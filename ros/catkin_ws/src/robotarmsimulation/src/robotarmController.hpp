#ifndef ROBOTARMCONTROLLER_H_
#define ROBOTARMCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <cstring>
#include <string>
#include <vector>

#define MINSIMULATEDDEGREES -3.14
#define MAXSIMULATEDDEGREES 3.14
#define MINPULSEWIDTH 500
#define MAXPULSEWIDTH 2500

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
    void initializeValues();

    // Robotarm position variables
    std::vector<std::string> mJointNames;
    std::vector<double> mJointPositions;

    void sendCurrentStateToVisualizer();

    void handleCommand(std::string aCommand);

    void robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand);

      /**
     * @brief Maps the value from the input range to the output range
     * @param aDegree - The value to convert
     * @param aInMin - The minimum value of the input range
     * @param aInMax - The maximum value of the input range
     * @param aOutMin - The minimum value of the output range
     * @param aOutMax - The maximum value of the output range 
     * @return unsigned int - The converted value
     */
    double mapValues(double aDegree, double aInMin, double aInMax, double aOutMin, double aOutMax) const;
};

#endif