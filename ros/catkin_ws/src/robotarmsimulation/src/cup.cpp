#include "cup.hpp"

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

    uint32_t shape = visualization_msgs::Marker::CYLINDER;

    while (ros::ok())
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();

        marker.ns = "cup";
        marker.id = 0;

        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;
        marker.lifetime = ros::Duration();

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub.publish(marker);
    }
}