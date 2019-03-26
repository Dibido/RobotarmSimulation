#include "robotarmController.hpp"
#include "cup.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotarmController");
  RobotarmController lRobotarmController;

  // Cup c("test");

  ros::spin();
  return 0;
}

RobotarmController::RobotarmController() :
  mJointNames{"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand"}, 
  mJointPositions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
  initializeCommunication();
  initializeValues();
}

RobotarmController::~RobotarmController()
{
}

void RobotarmController::initializeCommunication()
{
  mRobotarmCommandSubscriber = mNodeHandler.subscribe("robotarmCommand", 1000, &RobotarmController::robotarmCommandCallback, this);
  mRobotarmPublisher = mNodeHandler.advertise<sensor_msgs::JointState>("joint_states", 1000);
}

void RobotarmController::initializeValues()
{
}

void RobotarmController::sendCurrentStateToVisualizer()
{
  while (ros::ok())
  {
    sensor_msgs::JointState lMessage;
    lMessage.header.frame_id = "/base_link";
    lMessage.header.stamp = ros::Time::now();
    lMessage.name = mJointNames;
    lMessage.position = mJointPositions;
    mRobotarmPublisher.publish(lMessage);
    // std::cout << lMessage << std::endl;
  }
}

void RobotarmController::handleCommand(std::string aCommand)
{
  // Parse command, always parse for all the servo's
  std::string lOriginalString = aCommand;
  // Get the servo id's and pulsewidth
  std::string lSubstring;
  std::vector<double> lJointPulseWidths;
  lSubstring = lOriginalString.substr(lOriginalString.find("#") + 1);
  lJointPulseWidths.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  lJointPulseWidths.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  lJointPulseWidths.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  lJointPulseWidths.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  lJointPulseWidths.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  lJointPulseWidths.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("T")).c_str()));
  // Get the time
  unsigned int lTimeCommand = atoi(lSubstring.substr(lSubstring.find("T") + 1).c_str());
  std::cout << "time : " <<  lTimeCommand << std::endl;
  // Convert to angles
  for(int i = 0; i < lJointPulseWidths.size(); i++)
  {
    std::cout << lJointPulseWidths.at(i) << " : " << mapValues(lJointPulseWidths.at(i), MINPULSEWIDTH, MAXPULSEWIDTH, MINSIMULATEDDEGREES, MAXSIMULATEDDEGREES) << std::endl;
    lJointPulseWidths.at(i) = mapValues(lJointPulseWidths.at(i), MINPULSEWIDTH, MAXPULSEWIDTH, MINSIMULATEDDEGREES, MAXSIMULATEDDEGREES);
  }
  // Publish to joint_states
  for(int i = 0; i < lJointPulseWidths.size(); i++)
  {
    mJointPositions.at(i) = lJointPulseWidths.at(i);
  }
  sendCurrentStateToVisualizer();
}

void RobotarmController::robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand)
{
  ROS_INFO("Recieved robotarmCommand : %s", aRobotarmCommand->data.c_str());
  handleCommand(aRobotarmCommand->data);
}

double RobotarmController::mapValues(double aDegree, double aInMin, double aInMax, double aOutMin, double aOutMax) const
{
  return (aDegree - aInMin) * (aOutMax - aOutMin) / (aInMax - aInMin) + aOutMin;
}
