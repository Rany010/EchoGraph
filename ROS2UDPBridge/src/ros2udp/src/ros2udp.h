#ifndef ROS2UDP_H
#define ROS2UDP_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

class Ros2UDP
{
public:
    Ros2UDP(ros::NodeHandle& nh);
    ~Ros2UDP();

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    int sockfd_;
    struct sockaddr_in server_addr_;
};

#endif // ROS2UDP_H
