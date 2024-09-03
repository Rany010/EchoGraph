#include "logicalpro.h"

LogicalPro::LogicalPro() { initSocket(); }

void LogicalPro::startSlot() {
  stopRequested = false;
  recvDataThreadNew();
}

void LogicalPro::initSocket() {
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "Error: Could not create socket" << std::endl;
    exit(EXIT_FAILURE);
  }

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(2002);

  if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Error: Could not bind socket" << std::endl;
    close(sockfd);
    exit(EXIT_FAILURE);
  }

  std::cout << "Socket initialized and listening on port 2002" << std::endl;
}

int flag_first2 = 0;
void LogicalPro::recvDataThreadNew() {
  const size_t BUFFER_SIZE = 65535;
  const int POINT_STEP = 160;
  const int X_OFFSET = 0;
  const int Y_OFFSET = 4;
  const int Z_OFFSET = 8;

  char buffer[BUFFER_SIZE];

  while (!stopRequested) {
    ssize_t received_size =
        recvfrom(sockfd, buffer, BUFFER_SIZE, 0,
                 (struct sockaddr *)&client_addr, &client_addr_len);
    if (received_size < 0) {
      std::cerr << "Error: Failed to receive data" << std::endl;
      break;
    }

    //    std::cout << "Received data from " << inet_ntoa(client_addr.sin_addr)
    //    << ":"
    //              << ntohs(client_addr.sin_port) << std::endl;
    //    std::cout << "Received data length: " << received_size << std::endl;

    int num_points = received_size / POINT_STEP;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_xyzrgb->width = num_points;
    cloud_xyzrgb->height = 1;
    cloud_xyzrgb->points.resize(cloud_xyzrgb->width * cloud_xyzrgb->height);
    for (int i = 0; i < num_points; ++i) {
      std::memcpy(&cloud_xyzrgb->points[i].x,
                  buffer + i * POINT_STEP + X_OFFSET, sizeof(float));
      std::memcpy(&cloud_xyzrgb->points[i].y,
                  buffer + i * POINT_STEP + Y_OFFSET, sizeof(float));
      std::memcpy(&cloud_xyzrgb->points[i].z,
                  buffer + i * POINT_STEP + Z_OFFSET, sizeof(float));

      cloud_xyzrgb->points[i].r = static_cast<uint8_t>(255);
      cloud_xyzrgb->points[i].g = static_cast<uint8_t>(255);
      cloud_xyzrgb->points[i].b = static_cast<uint8_t>(255);
      //      cloud_xyzrgb->points[i].r = static_cast<uint8_t>(rand() % 256);
      //      cloud_xyzrgb->points[i].g = static_cast<uint8_t>(rand() % 256);
      //      cloud_xyzrgb->points[i].b = static_cast<uint8_t>(rand() % 256);
    }

    emit updatePointCloudSig(cloud_xyzrgb);
  }

  close(sockfd);
}
