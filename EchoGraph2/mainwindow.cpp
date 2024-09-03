#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
      vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
                                                   "viewer", false));

  // 获取 interactor 并进行设置
  view->setupInteractor(ui->guiwidget->renderWindow()->GetInteractor(),
                        ui->guiwidget->renderWindow());

  ui->guiwidget->setRenderWindow(view->getRenderWindow());

  logical_thread = nullptr;
  logical_pro = new LogicalPro();

  initConn();
  start_logical_thread();
}

MainWindow::~MainWindow() {
  delete ui;
  exit_logical_thread();
}

void MainWindow::initConn() {
  connect(this, &MainWindow::dynamicShowSig, logical_pro,
          &LogicalPro::startSlot);
  connect(logical_pro, &LogicalPro::updatePointCloudSig, this,
          &MainWindow::updatePointCloudSlot, Qt::QueuedConnection);
}

void MainWindow::on_pushButton_clicked() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
                                                  "Open PCD files(*.pcd)");
  if (fileName == "")
    return;
  pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
  view->addPointCloud(cloud, "cloud");
  view->resetCamera();
  ui->guiwidget->renderWindow()->Render(); // 强制渲染
}

void MainWindow::start_logical_thread() {
  if (logical_thread == nullptr) {
    logical_thread = new QThread;
  }

  logical_pro->moveToThread(logical_thread);
  logical_thread->start();
}

void MainWindow::exit_logical_thread() {
  if (logical_thread && logical_thread->isRunning()) {
    logical_pro->stop();
    logical_thread->exit();
    logical_thread->wait(); // 等待线程退出
    delete logical_thread;
    logical_thread = nullptr;
  }
}

void MainWindow::on_pushButton_2_clicked() { emit dynamicShowSig(); }

void MainWindow::updatePointCloudSlot(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb) {
  showPointCloud(cloud_xyzrgb);
}

void MainWindow::showPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb) {
  view->removeAllPointClouds();
  view->addPointCloud<pcl::PointXYZRGB>(cloud_xyzrgb, "sample cloud");
  view->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");

  view->getRenderWindow()->Render();
}
