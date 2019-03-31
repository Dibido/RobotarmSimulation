#include "cup.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cup");

  Cup c("test");
  c.handleCollision();

  return 0;
}

Cup::Cup(std::string aTopic)
{
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  publishCup();
}

Cup::~Cup()
{
}

void Cup::publishCup()
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_link";
  static_transformStamped.child_frame_id = "cup";
  static_transformStamped.transform.translation.x = cupPosX;
  static_transformStamped.transform.translation.y = cupPosY;
  static_transformStamped.transform.translation.z = cupPosZ;
  tf2::Quaternion quat;

  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
}

void Cup::showCup(COLORS::ColorState color)
{
  // std::cout << "Showing" << std::endl;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "cup";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cupObject";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = CUP_SIZE;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = CUP_SIZE;
  marker.scale.y = CUP_SIZE;
  marker.scale.z = CUP_SIZE * 2;
  marker.lifetime = ros::Duration();

  setColor(color, marker);

  marker_pub.publish(marker);
}

void Cup::handleCollision()
{
  COLORS::ColorState color = COLORS::RED;

  tf::Vector3 gripperOffset;
  ros::Rate rate(10.0);
  while (n.ok())
  {

    tf::StampedTransform leftGripper;
    tf::StampedTransform rightGripper;
    tf::StampedTransform newPosLeft;
    tf::StampedTransform newPosRight;

    try
    {
      listener.lookupTransform("/cup", "/gripper_left", ros::Time(0), leftGripper);
      listener.lookupTransform("/cup", "/gripper_right", ros::Time(0), rightGripper);
      listener.lookupTransform("/base_link", "/gripper_left", ros::Time(0), newPosLeft);
      listener.lookupTransform("/base_link", "/gripper_right", ros::Time(0), newPosRight);

      // std::cout << "X " << newPos.getOrigin().x() <<std::endl;
      // std::cout << "Y " << newPos.getOrigin().y() <<std::endl;


      if (isOpbjectInGripper(leftGripper) && isOpbjectInGripper(rightGripper) )
      {
        std::cout << "Z " << newPosLeft.getOrigin().x() <<std::endl;

        cupPosY = (newPosLeft.getOrigin().y() + newPosRight.getOrigin().y()) / 2;
        cupPosX = (newPosLeft.getOrigin().x() + newPosRight.getOrigin().x()) / 2;
        cupPosZ = (newPosLeft.getOrigin().z() + newPosRight.getOrigin().z()) / 2;
        
        cupPosY -= gripperOffset.y();
        cupPosX -= gripperOffset.x();
        cupPosZ -= gripperOffset.z()+0.05; //TODO remove the 0.05

        publishCup();
        color = COLORS::GREEN;
      }
      else
      {
        gripperOffset = leftGripper.getOrigin();
        std::cout << gripperOffset.z() << std::endl;
        color = COLORS::RED;
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    showCup(color);
    rate.sleep();
  }
}

void Cup::setColor(COLORS::ColorState color, visualization_msgs::Marker &marker)
{
  switch (color)
  {
  case COLORS::ColorState::RED:
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    break;

  case COLORS::ColorState::GREEN:
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    break;
  }
}

bool Cup::isOpbjectInGripper(tf::StampedTransform& object)
{
  const float widthMargin = CUP_SIZE - GRIPPER_DEPTH;
  const float heightMargin = (CUP_SIZE*2) - GRIPPER_DEPTH;

  std::cout << "test " << object.getOrigin().z() <<std::endl;

  return (object.getOrigin().y() > widthMargin * -1 && object.getOrigin().y() < widthMargin)
          && (object.getOrigin().x() > widthMargin * -1 && object.getOrigin().x() < widthMargin)
          && (object.getOrigin().z() > heightMargin * -1 && object.getOrigin().z() < heightMargin);
}