/**
 * @file pointcloudviewer.h
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

#include <QMainWindow>
#include <QContextMenuEvent>

#include <set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

namespace Ui {
class PointcloudViewer;
}

class PointcloudViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PointcloudViewer(std::vector<double> cameraParams , QWidget *parent = 0);
    ~PointcloudViewer();
    std::vector<double> cameraParams_;
public slots:
    void showPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc, int index = 0);
    void showPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc1,
                        pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc2,
                        const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& corners = {},
                        const std::string& cloud1_name = "filtered points",
                        const std::string& cloud2_name = "plane model" , int index = 1);
    void showCoordinateSystem(const Eigen::Affine3f& tf, int index, double scale=1.0);
    void resetViewer();
protected:
    void contextMenuEvent(QContextMenuEvent *event) override;

private slots:
    void on_actionSave_PointCloud_triggered();

private:
    Ui::PointcloudViewer *ui;
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer_;
    std::set<int> cloud_names_;
    std::set<int> frame_names_;
    bool line_flag = false;
};

#endif // POINTCLOUDVIEWER_H
