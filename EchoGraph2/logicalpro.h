#ifndef LOGICALPRO_H
#define LOGICALPRO_H

#include <QCoreApplication>
#include <QProcess>
#include <atomic>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LogicalPro : public QObject {
  Q_OBJECT
public:
  LogicalPro();

  bool testThread();
  void stop() { stopRequested = true; }

public slots:
  void startSlot();

public:
signals:
  void updatePointCloudSig(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb);

private:
  void initSocket();
  void recvDataThreadNew();
  int sockfd;
  struct sockaddr_in server_addr, client_addr;
  socklen_t client_addr_len = sizeof(client_addr);

  std::atomic<bool> stopRequested;
};

#endif // LOGICALPRO_H
