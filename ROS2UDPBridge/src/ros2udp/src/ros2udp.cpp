#include "ros2udp.h"

Ros2UDP::Ros2UDP(ros::NodeHandle& nh) : nh_(nh)
{
    // 初始化UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        ROS_ERROR("Failed to create socket");
        ros::shutdown();
        return;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(2002);
    inet_pton(AF_INET, "0.0.0.0", &server_addr_.sin_addr);

    // 订阅radar_left_topic
    point_cloud_sub_ = nh_.subscribe("/pc2_msg_front_mid", 10, &Ros2UDP::pointCloudCallback, this);
}

Ros2UDP::~Ros2UDP()
{
    close(sockfd_);
}

void Ros2UDP::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ssize_t sent_size = sendto(sockfd_, msg->data.data(), msg->data.size(), 0,
                               (struct sockaddr *)&server_addr_, sizeof(server_addr_));
    if (sent_size < 0)
    {
        ROS_ERROR("Failed to send UDP packet");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ros2UDP");
    ros::NodeHandle nh;

    Ros2UDP ros2udp(nh);

    ros::spin();
    return 0;
}