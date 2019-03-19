#include "robotarmController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotarmController");
  RobotarmController lRobotarmController;
  ros::spin();
  return 0;
}

RobotarmController::RobotarmController()
{
  mRobotarmCommandSubscriber = mNodeHandler.subscribe("robotarmCommand", 1000, &RobotarmController::robotarmCommandCallback, this);
  mRobotarmPublisher = mNodeHandler.advertise<sensor_msgs::JointState>("joint_states", 1000);
}

RobotarmController::~RobotarmController()
{

}

void RobotarmController::robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand)
{
  ROS_INFO("Recieved robotarmCommand : %s", aRobotarmCommand->data.c_str());
}
