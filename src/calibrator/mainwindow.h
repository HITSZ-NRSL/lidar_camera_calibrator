/**
 * @file mainwindow.h
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pointcloudviewer.h"
#include "imageviewer.h"

#include "json/json.hpp"
#include "data_reader.h"

#include "image_polygon.h"
#include "pointcloud_polygon.h"

//#define VERBOSE_INFO

/* Private typedef -----------------------------------------------------------*/
using PCC = pcl::PointCloud<pcl::PointXYZRGB>;
using PCI = pcl::PointCloud<pcl::PointXYZI>;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
protected:
    void closeEvent(QCloseEvent *);
private:
    /**
     * @brief a polygon
     */
    struct Polygon
    {
        ImagePolygon::Ptr img;
        PointcloudPolygon::Polygon3D::ConstPtr pc;

        Polygon():img(nullptr), pc(nullptr){}
    };

    class Color
    {
    public:
        using rgb = std::array<uint8_t,3>;
        using rgbs = std::vector<rgb>;
        static rgb HSL2RGB(double H, double S, double L)
        {
            rgb res{{0,0,0}};

            H = H<0 || H>360 ? 180:H;
            S = S<0 || S>1 ? 0.5:S;
            L = L<0 || L>1 ? 0.5:L;


            double C = (1-std::abs(2*L-1))*S;
            double X = C*(1-std::abs(std::fmod(H/60, 2) -1));
            double m = L - C/2;

            double RR, GG, BB;

            if(H <60)
            {
                RR = C;
                GG = X;
                BB = 0;
            }
            else if(H <120)
            {
                RR = X;
                GG = C;
                BB = 0;
            }
            else if(H <180)
            {
                RR = 0;
                GG = C;
                BB = X;
            }
            else if(H <240)
            {
                RR = 0;
                GG = X;
                BB = C;
            }
            else if(H < 300)
            {
                RR = X;
                GG = 0;
                BB = C;
            }
            else
            {
                RR = X;
                GG = 0;
                BB = C;
            }

            res[0] = static_cast<uint8_t>((RR+m)*255);
            res[1] = static_cast<uint8_t>((GG+m)*255);
            res[2] = static_cast<uint8_t>((BB+m)*255);
            return res;
        };
        static rgbs get_rgbs(uint32_t n, double S=0.5, double L=0.5 )
        {
            n = !n ? 1:n;
            rgbs res(n);
            double step = 360/n;
            double H = 0;
            for(uint32_t i=0; i<n; i++, H+= step)
            {
                res[i] = HSL2RGB(H,S,L);
            }

            return res;
        }

    };

    /**
     * @brief a sensor data frame
     */
    struct SensorData
    {
        bool img_good;  ///< true when img_corners are not null
        bool pc_good;   ///< true when pc_corners are not null
        bool pc_valid;   ///< true when pc in not nullptr
        uint32_t id;
        uint32_t pid;
        std::shared_ptr<cv::Mat> img;   ///< undistorted image
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_plane;
        std::shared_ptr<cv::Mat> img_marked;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_marked;
        Eigen::Vector4d pc_plane_coef;
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> corners_3d;
        std::shared_ptr<cv::Mat> img_proj;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_proj, pc_proj_cam;
        Polygon polygon;

        SensorData(uint32_t index):img_good(false), pc_good(false), pc_valid(false),
            id(index),
            img(nullptr), pc(nullptr),
            img_marked(nullptr), pc_marked(nullptr),
            img_proj(nullptr), pc_proj(nullptr), pc_proj_cam(nullptr), polygon(){}
        bool good()
        {
            return (img_good && pc_good);
        }
    };

    std::unique_ptr<ImagePolygon> imgply_;
    std::unique_ptr<PointcloudPolygon> pcply_;
    const int size_ = 4; // for rectangle
    Eigen::Matrix3d K_;
    Eigen::Matrix<double,5,1> D_;
    Eigen::Matrix<double,3,1> tag_size_;
    Ui::MainWindow *ui;

    QString config_path_;
    nlohmann::json js_;

    bool is_calibrated_;
    Eigen::Matrix4d T_;
    uint32_t sid_;
    std::unordered_set<uint32_t> skip_ids_;
    QString data_root_;

    std::shared_ptr<cv::Mat> img_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_;

    std::unique_ptr<DataReader> data_reader_;
    std::unique_ptr<PointcloudViewer> pc_viewer_;
    std::unique_ptr<ImageViewer> img_viewer_;
    std::vector<SensorData> sensor_data_;

    //GUI parameters
    QString defaultBGDColor = "background-color: rgb(238, 238, 236);";
    QString clickedBGDColor = "background-color: rgb(186, 189, 182);";
    bool is_selecting ;
    bool is_select_end ;

    void updateLabels();
    bool processData(bool is_check = true);
    void setEnabledAll(bool status);
    void showCalibrateResult();
    void project(std::shared_ptr<cv::Mat> img_orig, std::shared_ptr<cv::Mat> img_proj,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_proj_cam);
private slots:
    void openConfig(const std::string &config_path);
    void openDataset(const QString &dataset_path);
    void on_actionOpen_Dataset_triggered();
    void on_extract_clicked();
    void on_next_pose_clicked();
    void on_skip_pose_clicked();
    void on_calibrate_clicked();

    void on_pick_points_start_clicked();
    void on_pick_points_end_clicked();
    void on_actionOpen_Config_triggered();

    void processSlider();
    void on_actionSave_Result_triggered();
};

#endif // MAINWINDOW_H
