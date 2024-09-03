#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[]) {
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
      "pcl::PointCloud<pcl::PointXYZRGB>::Ptr");

  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
