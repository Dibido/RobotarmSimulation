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
  static_transformStamped.transform.translation.x = CUP_POS_X;
  static_transformStamped.transform.translation.y = CUP_POS_Y;
  static_transformStamped.transform.translation.z = CUP_POS_Z;
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
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
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

  ros::Rate rate(10.0);
  while (n.ok())
  {

    tf::StampedTransform leftGripperTransform;

    try
    {
      echoListener.lookupTransform("/cup", "/gripper_left", ros::Time(0), leftGripperTransform);
      std::cout << "X" << leftGripperTransform.getOrigin().x() << std::endl;
      std::cout << "Y" << leftGripperTransform.getOrigin().y() << std::endl;
      std::cout << "Z" << leftGripperTransform.getOrigin().z() << std::endl;

      if (isOpbjectInGripper(leftGripperTransform))
        color = COLORS::GREEN;
      else
        color = COLORS::RED;
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
  return (object.getOrigin().y() > CUP_SIZE * -1 && object.getOrigin().y() < CUP_SIZE)
          && (object.getOrigin().x() > CUP_SIZE * -1 && object.getOrigin().x() < CUP_SIZE)
          && (object.getOrigin().z() > (CUP_SIZE*2) * -1 && object.getOrigin().z() < (CUP_SIZE*2));
}