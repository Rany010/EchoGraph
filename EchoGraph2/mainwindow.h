#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> //各种格式的点的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>

#include "logicalpro.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  void initConn();
  void showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb);

signals:
  void dynamicShowSig();

private slots:
  void on_pushButton_clicked();

  void on_pushButton_2_clicked();

public slots:
  void
  updatePointCloudSlot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb);

private:
  void start_logical_thread();
  void exit_logical_thread();
  LogicalPro *logical_pro;
  QThread *logical_thread;

private:
  Ui::MainWindow *ui;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
};
#endif // MAINWINDOW_H
