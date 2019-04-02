#include "cup.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cup");

  if (argc >= 5)
  {
    float argX, argY, argZ;
    argX = atof(argv[1]);
    argY = atof(argv[2]);
    argZ = atof(argv[3]);
    std::string argName = std::string(argv[4]);
    Cup c(argName, argX, argY, argZ);
    c.handleCollision();
  }
  else
  {
    Cup c("cup");
    c.handleCollision();
  }

  return 0;
}

Cup::Cup(std::string aTopic, float aOriginalX, float aOriginalY, float aOriginalZ) : cupPosX(aOriginalX), cupPosY(aOriginalY), cupPosZ(aOriginalZ), topic(aTopic)
{
  pastFrameTime = ros::Time::now();
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  publishCup();
}

Cup::~Cup()
{
}

void Cup::publishCup() const
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_link";
  static_transformStamped.child_frame_id = topic;
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

void Cup::showCup(const COLORS::ColorState color) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = topic;
  marker.header.stamp = ros::Time::now();
  marker.ns = "cupObject_" + topic;
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
  ros::spinOnce();
}

void Cup::showMarker(const COLORS::ColorState color,const std::string &frameName,const float offset) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameName;
  marker.header.stamp = ros::Time::now();
  marker.ns = "marker_" + frameName;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = offset;
  marker.pose.position.z = GRIPPER_DEPTH;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.lifetime = ros::Duration();

  setColor(color, marker);

  marker_pub.publish(marker);
  ros::spinOnce();
}

void Cup::handleCollision()
{
  tf::Vector3 gripperOffset;
  ros::Rate rate(300.0);
  while (n.ok())
  {
    //Mark the grippers
    showMarker(COLORS::RED, FRAME_NAME_GRIPPER_RIGHT, GRIPPER_DEPTH);
    showMarker(COLORS::RED, FRAME_NAME_GRIPPER_LEFT, GRIPPER_DEPTH * -1);
    
    tf::StampedTransform leftGripper;
    tf::StampedTransform rightGripper;
    tf::StampedTransform newPosLeft;
    tf::StampedTransform newPosRight;
    tf::StampedTransform cup;

    try
    {
      std::string topicFrame = std::string("/") + topic;
      listener.lookupTransform(WORLD_OBJECT, topicFrame, ros::Time(0), cup);
      listener.lookupTransform(topicFrame, FRAME_NAME_GRIPPER_LEFT, ros::Time(0), leftGripper);
      listener.lookupTransform(topicFrame, FRAME_NAME_GRIPPER_RIGHT, ros::Time(0), rightGripper);
      listener.lookupTransform(WORLD_OBJECT, FRAME_NAME_GRIPPER_LEFT, ros::Time(0), newPosLeft);
      listener.lookupTransform(WORLD_OBJECT, FRAME_NAME_GRIPPER_RIGHT, ros::Time(0), newPosRight);

      if (isOpbjectInGripper(leftGripper) && isOpbjectInGripper(rightGripper))
      {
        cupPosY = (newPosLeft.getOrigin().y() + newPosRight.getOrigin().y()) / 2;
        cupPosX = (newPosLeft.getOrigin().x() + newPosRight.getOrigin().x()) / 2;

        auto newPosCupZ = ((newPosLeft.getOrigin().z() + newPosRight.getOrigin().z()) / 2) - gripperOffset.z() + 0.005;

        if (newPosCupZ > 0)
          cupPosZ = newPosCupZ;

        color = COLORS::GREEN;
      }
      else
      {
        gripperOffset = leftGripper.getOrigin();
        color = COLORS::RED;

        if (cup.getOrigin().z() > 0)
        {
          auto timePast = ros::Time::now() - pastFrameTime;
          float dropMulitplayer = timePast.toSec() / calculateFallingTime(cup);
          cupPosZ -= cup.getOrigin().z() * dropMulitplayer;
        }
      }

      publishCup();
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    

    showCup(color);

    pastFrameTime = ros::Time::now();
    rate.sleep();
  }
}

void Cup::setColor(const COLORS::ColorState color, visualization_msgs::Marker &marker) const
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

bool Cup::isOpbjectInGripper(const tf::StampedTransform &object) const
{
  const float widthMargin = CUP_SIZE - GRIPPER_DEPTH;
  const float heightMargin = (CUP_SIZE * 2) - GRIPPER_DEPTH;

  return (object.getOrigin().y() > widthMargin * -1 && object.getOrigin().y() < widthMargin) && (object.getOrigin().x() > widthMargin * -1 && object.getOrigin().x() < widthMargin) && (object.getOrigin().z() > heightMargin * -1 && object.getOrigin().z() < heightMargin);
}

float Cup::calculateFallingTime(const tf::StampedTransform &object) const
{
  const float FORCE_OF_GRAVITY = 9.5;

  float value = 2 * object.getOrigin().z() / FORCE_OF_GRAVITY;
  return sqrt(value);
}