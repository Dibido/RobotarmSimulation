#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotarmClient");
  // Create robotarmController publisher
  ros::NodeHandle lNodeHandler;
  ros::Publisher lRobotarmCommandPublisher = lNodeHandler.advertise<std_msgs::String>("robotarmCommand", 1000);
  // Publish commands
  if(ros::ok())
  {
    sleep(1);
    std_msgs::String msg;

    std::stringstream ss;
    ss << "#5P1600#10P750T2500\n";
    msg.data = ss.str();

    ROS_INFO("sending command : %s", msg.data.c_str());

    lRobotarmCommandPublisher.publish(msg);

    ros::spinOnce();
    sleep(1);
  }
  return 0;
}