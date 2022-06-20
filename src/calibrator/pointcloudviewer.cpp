/**
 * @file pointcloudviewer.cpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include "pointcloudviewer.h"
#include "ui_pointcloudviewer.h"

#include <QMenu>
#include <QDebug>


PointcloudViewer::PointcloudViewer(std::vector<double> cameraParams , QWidget *parent) :
    QMainWindow(parent),
    cameraParams_(cameraParams),
    ui(new Ui::PointcloudViewer)
{

    ui->setupUi(this);
    setWindowFlags(Qt::Window
                   | Qt::WindowMinimizeButtonHint
                   | Qt::WindowMaximizeButtonHint);

    viewer_ = std::unique_ptr<pcl::visualization::PCLVisualizer>(
                  new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setCameraPosition(cameraParams_[0] , cameraParams_[1] ,
                                cameraParams_[2] , cameraParams_[3] ,
                                cameraParams_[4] , cameraParams_[5] );

    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    viewer_->resetCamera();
    showCoordinateSystem(Eigen::Affine3f::Identity(), 0, 2);
}

PointcloudViewer::~PointcloudViewer()
{
    delete ui;
}


/**
 * @brief show(add/update) pointcloud in the viewer
 * @param [in] pc xyzrgb pointcloud pointer
 * @param [in] index pointcloud id
 */
void PointcloudViewer::showPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc, int index)
{
    std::string pc_name = std::string("cloud_")+std::to_string(index);
    if(cloud_names_.count(index))
    {
        // update
        viewer_->updatePointCloud(pc, pc_name);
    }
    else
    {
        //add
        cloud_names_.insert(index);
        viewer_->addPointCloud(pc, pc_name);
    }
    ui->qvtkWidget->update();
}

void PointcloudViewer::showPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc1,
                    pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc2,
                    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& corners,
                    const std::string& cloud1_name,const std::string& cloud2_name , int index)
{
    if(corners.size() != 0){    // reset when corners are not empty
        resetViewer();
    }
    if(corners.size() == 0 && line_flag ){ 
        line_flag = false;
        resetViewer();
    }
    int v1 = 0;
    int v2 = 3;
    if(cloud_names_.count(index))
    {
        // update
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc1);
        viewer_->updatePointCloud<pcl::PointXYZRGB>(pc1 ,rgb ,"sample cloud1");
        viewer_->updatePointCloud<pcl::PointXYZI>(pc2 , "sample cloud2");
    }
    else
    {
        //add
        cloud_names_.insert(index);
        // creat window 1
        viewer_->createViewPort(0.0, 0.0, 0.5, 1.0,v1);  
        viewer_->setBackgroundColor (0.2, 0.2, 0.2, v1);    
        viewer_->addText(cloud1_name, 10, 10, 20, 0 , 0, 1,"v1 text", v1);  
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc1);
        viewer_->addPointCloud<pcl::PointXYZRGB> (pc1, rgb,"sample cloud1", v1);

        viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer_->setBackgroundColor (0.4, 0.4, 0.4, v2);
        viewer_->addText(cloud2_name, 10, 10, 20, 0 , 0, 1,"v2 text", v2);
        viewer_->addPointCloud<pcl::PointXYZI> (pc2, "sample cloud2", v2);

        viewer_->addCoordinateSystem (1.0);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
        if(corners.size() != 0){
            assert( corners.size() == 4);
            Eigen::Vector4d corner1 = corners[0], corner2 = corners[1], corner3 = corners[2],corner4 = corners[3];
            pcl::PointXYZ p1(corner1.x(), corner1.y(), corner1.z());
            pcl::PointXYZ p2(corner2.x(), corner2.y(), corner2.z());
            pcl::PointXYZ p3(corner3.x(), corner3.y(), corner3.z());
            pcl::PointXYZ p4(corner4.x(), corner4.y(), corner4.z());
            viewer_->addLine<pcl::PointXYZ>(p1,p2,255,0,0,"1",v2);
            viewer_->addLine<pcl::PointXYZ>(p2,p3,255,0,0,"2",v2);
            viewer_->addLine<pcl::PointXYZ>(p3,p4,255,0,0,"3",v2);
            viewer_->addLine<pcl::PointXYZ>(p4,p1,255,0,0,"4",v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,"1",v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,"2",v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,"3",v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,"4",v2);
            viewer_->addText3D<pcl::PointXYZ>("1",p1,0.15,0,1,0,"text 1",v2);
            viewer_->addText3D<pcl::PointXYZ>("2",p2,0.15,0,1,0,"text 2",v2);
            viewer_->addText3D<pcl::PointXYZ>("3",p3,0.15,0,1,0,"text 3",v2);
            viewer_->addText3D<pcl::PointXYZ>("4",p4,0.15,0,1,0,"text 4",v2);
            line_flag = true;
        }

    }
    ui->qvtkWidget->update();


}

/**
 * @brief show coordinate in the viewer
 * @param [in] tf    the coordinate transformation
 * @param [in] index the coordinate id
 * @param [in] scale the scale, default 1.0
 */
void PointcloudViewer::showCoordinateSystem(const Eigen::Affine3f& tf, int index, double scale)
{
    std::string frame_name = std::string("frame_") + std::to_string(index);
    if(frame_names_.count(index))
    {
        viewer_->updateCoordinateSystemPose(frame_name, tf);
    }
    else
    {
        frame_names_.insert(index);
        viewer_->addCoordinateSystem(scale, tf, frame_name);
    }
}


void PointcloudViewer::contextMenuEvent(QContextMenuEvent *event)
{
    qDebug() << "right click";
    QMenu menu(this);
    menu.addAction(ui->actionSave_PointCloud);
    menu.exec(event->globalPos());
}

void PointcloudViewer::on_actionSave_PointCloud_triggered()
{
    qDebug() << "right click";
}

void PointcloudViewer::resetViewer()
{
    pcl::visualization::Camera camera_before;
    viewer_->getCameraParameters(camera_before);
    viewer_.reset();
    viewer_ = std::unique_ptr<pcl::visualization::PCLVisualizer>(
                  new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setCameraParameters(camera_before);
    // viewer_->setCameraPosition(cameraParams_[0] , cameraParams_[1] ,
    //                             cameraParams_[2] , cameraParams_[3] ,
    //                             cameraParams_[4] , cameraParams_[5] );

    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    // viewer_->resetCamera();
    showCoordinateSystem(Eigen::Affine3f::Identity(), 0, 2);
    cloud_names_.erase(1);
}
