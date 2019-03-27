#include "cup.hpp"

std::string static_name;

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

    static_name = "base_link";
    //Frame
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "cup";
    static_transformStamped.child_frame_id = static_name;
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
    ROS_INFO("Spinning until killed publ");
}

void Cup::showCup()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/cup";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cup";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;
    marker.lifetime = ros::Duration();

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;


    // tf::StampedTransform leftGripperTransform;
    // echoListener.lookupTransform(source_frameid, gripperLeftTarget, ros::Time(), leftGripperTransform);
    // tf::StampedTransform rightGripperTransform;
    // echoListener.lookupTransform(source_frameid, gripperRightTarget, ros::Time(), rightGripperTransform);

    // std::cout << leftGripperTransform.getOrigin() << std::endl;
    // std::cout << rightGripperTransform.getOrigin() << std::endl;

    marker_pub.publish(marker);
}