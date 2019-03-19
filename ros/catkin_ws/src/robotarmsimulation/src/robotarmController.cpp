#include "robotarmController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotarmController");
  RobotarmController lRobotarmController;
  ros::spin();
  return 0;
}

RobotarmController::RobotarmController() : 
  mJointNames{"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand"}, 
  mJointPositions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
  initializeCommunication();
  initializeVales();
}

RobotarmController::~RobotarmController()
{
}

void RobotarmController::initializeCommunication()
{
  mRobotarmCommandSubscriber = mNodeHandler.subscribe("robotarmCommand", 1000, &RobotarmController::robotarmCommandCallback, this);
  mRobotarmPublisher = mNodeHandler.advertise<sensor_msgs::JointState>("joint_states", 1000);
}

void RobotarmController::initializeVales()
{
}

void RobotarmController::sendCurrentStateToVisualizer()
{
  sensor_msgs::JointState lMessage;
  lMessage.header.stamp.sec = 0;
  lMessage.name = mJointNames;
  lMessage.position = mJointPositions;
  mRobotarmPublisher.publish(lMessage);
  std::cout << lMessage << std::endl;
}

void RobotarmController::handleCommand(std::string aCommand)
{
  // Parse command, always parse for all the servo's

  // Convert to angles
  // Publish to joint_states
  mJointPositions.at(0) = 1.0;
  sendCurrentStateToVisualizer();
}

void RobotarmController::robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand)
{
  ROS_INFO("Recieved robotarmCommand : %s", aRobotarmCommand->data.c_str());
  handleCommand(aRobotarmCommand->data);
}
