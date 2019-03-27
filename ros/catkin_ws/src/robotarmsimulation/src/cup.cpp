#include "cup.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cup");

  Cup c("test");
  c.showCup();
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
  // std::cout << "Publishing" << std::endl;
  // tf::TransformBroadcaster static_broadcaster;
  // static_name = "base_link";
  // geometry_msgs::TransformStamped static_transformStamped;
  // static_transformStamped.header.stamp = ros::Time::now();
  // static_transformStamped.header.frame_id = "cup";
  // static_transformStamped.child_frame_id = static_name;
  // static_transformStamped.transform.translation.x = 1;
  // static_transformStamped.transform.translation.y = 1;
  // static_transformStamped.transform.translation.z = 0;
  // tf::Quaternion quat;
  // quat.setRPY(0, 0, 0);
  // static_transformStamped.transform.rotation.x = quat.x();
  // static_transformStamped.transform.rotation.y = quat.y();
  // static_transformStamped.transform.rotation.z = quat.z();
  // static_transformStamped.transform.rotation.w = quat.w();
  // static_broadcaster.sendTransform(static_transformStamped);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base_link";
    static_transformStamped.child_frame_id = "cup";
    static_transformStamped.transform.translation.x = 1;
    static_transformStamped.transform.translation.y = 1;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;

    quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

void Cup::showCup()
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
  marker.pose.position.z = 0.05;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.10;
  marker.lifetime = ros::Duration();

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker_pub.publish(marker);
}

void Cup::handleCollision()
{
  // tf::TransformListener listener;

  // // ros::Rate rate(10.0);
  // // while (n.ok()){
  //   tf::StampedTransform transform;
  //   try
  //   {
  //     listener.lookupTransform("/cup", "/gripper_left", ros::Time(0), transform);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_ERROR("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //   }

  ros::Rate rate(10.0);
  while (n.ok()){
    showCup();

    

    tf::StampedTransform leftGripperTransform;
    
    try{
      echoListener.lookupTransform("/cup", "/gripper_left", ros::Time(0), leftGripperTransform);
      std::cout << "X" << leftGripperTransform.getOrigin().x() << std::endl;
      std::cout << "Y" << leftGripperTransform.getOrigin().y() << std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    


  //   // turtlesim::Velocity vel_msg;
  //   // vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
  //   //                             transform.getOrigin().x());
  //   // vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
  //   //                             pow(transform.getOrigin().y(), 2));

    rate.sleep();
   }
}