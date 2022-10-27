/**
 * @file mainwindow.cpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QDebug>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include <QCloseEvent>
#include <QThread>
#include <QInputDialog>
#include <QLineEdit>
#include <QDateTime>
#include <QTimer>

#include <string>
#include <set>
#include <unordered_set>
#include <fstream>

#include "single_tagtest.h"
#include "core/SolvePnP.hpp"
#include "core/PCFeatureExtra.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    is_calibrated_(false),
    sid_(INT_MAX),
    img_(nullptr), pc_(nullptr),
    data_reader_(nullptr),
    is_selecting(false),
    is_select_end(false)
{
    ui->setupUi(this);
    T_ = Eigen::Matrix4d::Identity();
    if(argc < 2) {
        QMessageBox::warning(this, tr("Warn"), tr("You need to open the config and dataset manually "));
    } else {
        config_path_ = QString(argv[1]);
        config_path_ += "/config.json";
//        cout << config_path_.toStdString() << endl;
        openConfig(config_path_.toStdString());
        openDataset(QString(argv[1]));
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent * event)
{
    if(img_viewer_)
    {
        img_viewer_->close();
        img_viewer_.release();
    }
    if(pc_viewer_)
    {
        pc_viewer_->close();
        img_viewer_.release();
    }

    event->accept();
}

void MainWindow::on_actionOpen_Config_triggered()
{
    // read config file by dialog box
    config_path_ = QFileDialog::getOpenFileName(this,
                   tr("Open File"),
                   ".",
                   tr("Config JSON Files(*.json)"));

    if(config_path_.isEmpty())
    {
        QTimer::singleShot(0,this, &QApplication::quit);
        return;
    }
    openConfig(config_path_.toStdString() );
}

void MainWindow::on_actionOpen_Dataset_triggered()
{
    if(config_path_.toStdString() == std::string() )//config file hasn't been opened.
    {
        QMessageBox::warning(this, tr("Warn"), tr("Please open config first!"));
        return;
    }
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                  QDir::homePath(),
                  QFileDialog::ShowDirsOnly);

    openDataset(dir);
}

void MainWindow::on_extract_clicked(){
    if(is_calibrated_)
    {
//    ui->quick_next_pose->setText(tr("Previous"));

    } else {
        if( sensor_data_.empty() ) return;

        if( !sensor_data_.back().pc_valid )  // ensure that board point cloud is valid 
        {
            QMessageBox::warning(this, tr("Error"), tr("Current pc data is empty, adjust it!"));
            return;
        }
        
        // extract corners of pc plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*sensor_data_.back().pc_plane, *pc_plane_cloud);
        Eigen::Matrix4d tf_lidar_to_target = extractPCFeature(pc_plane_cloud, data_reader_->tag_size_);
        Eigen::Matrix4d corners_target;
        corners_target << 0,-data_reader_->tag_size_[1]/2,-data_reader_->tag_size_[0]/2, 1,
                        0,-data_reader_->tag_size_[1]/2,data_reader_->tag_size_[0]/2, 1,
                        0,data_reader_->tag_size_[1]/2,data_reader_->tag_size_[0]/2, 1,
                        0,data_reader_->tag_size_[1]/2,-data_reader_->tag_size_[0]/2, 1;
        Eigen::Matrix4d corners_lidar = tf_lidar_to_target.inverse() * corners_target.transpose();
        // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> corners_lidar_vector;
        auto& last = sensor_data_.back();
        last.corners_3d.resize(4);
        for(int i = 0; i < 4; ++i){
            last.corners_3d[i] = corners_lidar.col(i); 
        }
        auto compare_rule = [](Eigen::Vector4d p1, Eigen::Vector4d p2){
            return p1.z() > p2.z();// Descending
        };
        static std::string lidar_pose = js_["lidar_pose"];
        if(lidar_pose == "ascend") {
            std::sort(last.corners_3d.rbegin(), last.corners_3d.rend(), compare_rule);
        } else if(lidar_pose == "descend") {
            std::sort(last.corners_3d.begin(), last.corners_3d.end(), compare_rule);
        } else {
            assert(0 && "Invalid Lidar Pose Setting.");
        }
        std::swap(last.corners_3d[2], last.corners_3d[3]);
        // visualize the four corners
        pc_viewer_->showPointcloud(last.pc_marked , last.pc_plane, last.corners_3d);
        sensor_data_.back().pc_good = true;
    }
}

void MainWindow::on_skip_pose_clicked(){
    if(is_calibrated_)
    {
        // The button will become 'Previous Pose' if calibration is finished.
        if(sid_ > 0) {
            --sid_;
        }
        else {
            sid_ = sensor_data_.size() - 1;
        }
        showCalibrateResult();
        updateLabels();
        return;
    }

    skip_ids_.insert(sid_);
    // A new frame of data will be processed when 'skip pose' is pressed
    if(processData())
    {
        if(is_select_end)
        {
            is_select_end = false;
            ui->pick_points_end->setStyleSheet(defaultBGDColor);
        }
    }
}

void MainWindow::on_next_pose_clicked()
{
    if(is_calibrated_)
    {
        //after calibration
        if(sid_ < sensor_data_.size()-1)
        {
            sid_++;
        }
        else
        {
            if(data_reader_->moveNext())
            {
                sensor_data_.emplace_back(data_reader_->getCurrentId());
                auto& last = sensor_data_.back();
                last.img = data_reader_->getImage();
                last.pc = data_reader_->getPointcloud();
                last.pc_marked.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
                last.pc_plane.reset(new pcl::PointCloud<pcl::PointXYZI>);
                if(!last.img )
                {
                    sensor_data_.pop_back();
                    QMessageBox::warning(this, tr("Error"), tr("Fail to read image"));
                    return;
                }
                if(!last.pc)
                {
                    sensor_data_.pop_back();
                    QMessageBox::warning(this, tr("Error"), tr("Fail to read pointcloud"));
                    return;
                }
                last.pid = INT_MAX; // invalid
                sid_ = uint32_t( sensor_data_.size()-1 );
            }
            else
            {
                // last one and fail to read more, so do noting
                sid_ = 0;
//                return;
            }
        }
        showCalibrateResult();
        updateLabels();

    }
    else
    {
        // A new frame of data will be processed when 'next pose' is pressed.
        if(processData())
        {
            if(is_select_end)
            {
                is_select_end = false;
                ui->pick_points_end->setStyleSheet(defaultBGDColor);
            }
        }
    }
}

void MainWindow::on_calibrate_clicked()
{
    std::unordered_set<uint32_t> valid_data_ids;
    for (int i = 0; i < sensor_data_.size(); ++i) {
        if(skip_ids_.count(i) || !sensor_data_[i].good()) {
            continue;
        }
        valid_data_ids.insert(i);
    }
    if(valid_data_ids.empty())
    {
        QMessageBox::warning(this, tr("Error"), tr("Not enough valid data to do calibration, at lease 1"));
        return;
    }
    setEnabledAll(false);
    setCursor(Qt::WaitCursor);

    //// calibrate
    // prepare calibration data
    Eigen::Matrix3Xd points_2d; 
    Eigen::Matrix4Xd points_3d; 
    for (const auto i : valid_data_ids)
    {
        Eigen::Matrix4Xd points_3d_tmp;
        points_3d_tmp.resize(4,points_3d.cols() + sensor_data_[i].corners_3d.size() );
        auto& p3ds= sensor_data_[i].corners_3d;
        points_3d_tmp << points_3d, p3ds[0], p3ds[1], p3ds[2], p3ds[3];
        points_3d = points_3d_tmp;
        // points_3d_tmp.insert(points_3d_tmp.end(), sensor_data_[i].corners_3d.begin(), sensor_data_[i].corners_3d.end() );

        Eigen::Matrix3Xd points_2d_tmp;
        auto& ply = sensor_data_[i].polygon;
        points_2d_tmp.resize(3,points_2d.cols() + ply.img->vertexs.cols());
        points_2d_tmp << points_2d,ply.img->vertexs;
        points_2d = points_2d_tmp;
    }
    double fx = data_reader_->K_.ptr<float>(0)[0];
    double fy = data_reader_->K_.ptr<float>(1)[1];
    double cx = data_reader_->K_.ptr<float>(0)[2];
    double cy = data_reader_->K_.ptr<float>(1)[2];
    T_ =  solvePnPbyG2O(points_2d, points_3d, fx, fy, cx, cy, false);
    cout << "\033[1;32mCalibrationResult\033[0m:\t" << "Calibration is finished with " << valid_data_ids.size() << " pairs of data." << endl;
    cout << "\033[1;32mCalibrationResult\033[0m:\t" << "The calibrated transformation from lidar to camera is:" << endl;
    cout << "Transformation matrix: \n" << T_.matrix() << endl;
    Eigen::Matrix3d rot_m_l2c =  T_.topLeftCorner(3,3);
    cout << "Qaternion(qx qy qz qw):\t" << Eigen::Quaterniond(rot_m_l2c).coeffs().transpose() << endl;
    cout << "Translation(tx ty tz):\t" << T_.topRightCorner(3,1).transpose() << endl;

    cout << "\033[1;32mCalibrationResult\033[0m:\t" << "The calibrated transformation from camera to lidar is:" << endl;
    cout << "Transformation matrix: \n" << T_.inverse().matrix() << endl;
    Eigen::Matrix3d rot_m_c2l =  T_.inverse().topLeftCorner(3,3);
    cout << "Qaternion(qx qy qz qw):\t" << Eigen::Quaterniond(rot_m_c2l).coeffs().transpose() << endl;
    cout << "Translation(tx ty tz):\t" << T_.inverse().topRightCorner(3,1).transpose() << endl;

    setCursor(Qt::ArrowCursor);
    setEnabledAll(true);
    is_calibrated_ = true;
    ui->skip_pose->setText(tr("Previous Pose"));
    ui->extract->setEnabled(false);
    ui->calibrate->setEnabled(false);

    showCalibrateResult();
    updateLabels();
}

void MainWindow::on_pick_points_start_clicked()
{
    if(!img_viewer_)
    {
        QMessageBox::warning(this, tr("Error"), tr("Please open dataset first!"));
        return;
    }
    img_viewer_->startPickPoints();
    auto& last = sensor_data_.back();
    img_viewer_->showImage(last.img);

    is_selecting = true;
    ui->pick_points_start->setText("Selecting, click again to reselect") ;
    ui->pick_points_start->setStyleSheet(clickedBGDColor);
    if(is_select_end)
    {
        is_select_end = false;
        ui->pick_points_end->setStyleSheet(defaultBGDColor);
    }
}

void MainWindow::on_pick_points_end_clicked()
{
    if(!img_viewer_)
    {
        QMessageBox::warning(this, tr("Error"), tr("Please open dataset first"));
        return;
    }
    is_select_end = true;
    ui->pick_points_end->setStyleSheet(clickedBGDColor);
    if(is_selecting)
    {
        ui->pick_points_start->setText("Start Selection") ;
        is_selecting = false;
        ui->pick_points_start->setStyleSheet(defaultBGDColor);
    }

    std::vector<cv::Point2d> pts;
    img_viewer_->getPickPoitns(pts);

    if(pts.size() != js_["size"])
    {
        QMessageBox::warning(this, tr("Error"), tr("Must choose ")
                             + QString::number(js_["size"].get<int>())
                             +tr(" points"));
        return;
    }

    auto compare_rule = [](cv::Point2d p1, cv::Point2d p2){
        return p1.y < p2.y;// ascending 
    };
    std::sort(pts.begin(), pts.end(), compare_rule);
    std::swap(pts[2],pts[3]);// ensure the sequence

    auto& last = sensor_data_.back();
    auto& ply = last.polygon;
    ply.img->InitPolygon(pts);
    ply.img->MarkImage(*last.img , *last.img_marked);

    last.img_good = true;

    img_viewer_->showImage(last.img_marked);

    updateLabels();
}

void MainWindow::on_actionSave_Result_triggered()
{
    if(config_path_.toStdString() == std::string() )//config file hasn't been opened.
    {
        QMessageBox::warning(this, tr("Warn"), tr("Please open config first!"));
        return;
    }
    else if(!img_viewer_){ //dataset hasn't been opened.
        QMessageBox::warning(this, tr("Warn"), tr("Please open dataset first!"));
        return;
    }
    // saved to config.json(including updated parameters)
    std::ofstream f(config_path_.toStdString());
    if(!f.good())
    {
        QMessageBox::warning(this, tr("Error"), tr("Fail to open config file"));
        return;
    }

#ifdef VERBOSE_INFO
    QString fp;
    auto& img_i = js_["img"]["vertexs"];
    img_i.clear();
    auto& pc_coef_i = js_["pc"]["plane_coef"];
    pc_coef_i.clear();
    int i=0;

    for (auto& sensor_data : sensor_data_)
    {
        std::vector<double> temp_u;
        std::vector<double> temp_v;
        std::vector<double> temp_pc_coef;
        auto& ply = sensor_data.polygon;
        // image 
        if(ply.img)
        {
            for(int j=0; j<js_["size"].get<int>(); j++)
            {
                temp_u.push_back(ply.img->vertexs(0,j));
                temp_v.push_back(ply.img->vertexs(1,j));
            }
            img_i.push_back(std::vector<std::vector<double>> {temp_u ,temp_v });
        }
        else
        {
            QMessageBox::warning(this, tr("Error"),
                                 tr("img Id ")+QString("%1").arg(i) + tr(" don't have features!"));
            f << js_.dump(4);
            f.close();
            return;
        }

        // point cloud
        fp = data_reader_->dataset_root_ + QDir::separator() + data_reader_->kPointcloudFolderName +
             QDir::separator() + QString("plane%1.pcd").arg(i);

        try {
            for(int j = 0; j < 4; j++)
            {
                temp_pc_coef.push_back(sensor_data.pc_plane_coef[j]);
            }
            pc_coef_i.push_back(temp_pc_coef);
            pcl::io::savePCDFileASCII (fp.toStdString(), *sensor_data.pc_plane);
        }
        catch (...) {
            std::cerr<<"plane point cloud " <<i<<" is empty!"<<std::endl;
        }
        i++;
    }
#endif

    auto& tf = js_["tf"];
    tf.clear();
    for(uint8_t i=0; i<4; i++)
    {
        tf.push_back(std::vector<double> {T_(i,0), T_(i,1), T_(i,2), T_(i,3)});
    }

    f << js_.dump(4);

    f.close();
    //Saved successfully
    QMessageBox::information(this, tr("Success"), tr("Saved to config.json successfully!\n"));

}

void MainWindow::openConfig(const string &config_path)
{
    if( config_path.empty() ) { cout << "Cofnig path is empty!" << endl; return;}
    std::ifstream f(config_path);
    if(!f.good())
    {
        f.close();
        QTimer::singleShot(0,this, &QApplication::quit);
        return;
    }

    try
    {
        f >> js_;
    }
    catch(nlohmann::json::parse_error& e)
    {
        std::cerr << e.what();
        f.close();
        QTimer::singleShot(0,this, &QApplication::quit);
        return;
    }
    imgply_.reset( new ImagePolygon( uint8_t(size_) ) );
    pcply_.reset( new PointcloudPolygon(js_["pc"], uint8_t(size_) ) );

    auto& flt = js_["pc"]["filter"];
    ui->angle_start_slide->setValue(static_cast<int>(flt["angle_start"]));
    ui->angle_size_slide->setValue(static_cast<int>(flt["angle_size"]));
    ui->max_distance_slide->setValue(static_cast<int>(10 * flt["max_distance"].get<double>()) );
    ui->min_distance_slide->setValue(static_cast<int>(10 * flt["min_distance"].get<double>()) );
    ui->floor_gap_slide->setValue(static_cast<int>(10 * flt["floor_gap"].get<double>()) );
    ui->max_distance_slide->setMaximum(static_cast<int>(10 * flt["max_distance_threshold"].get<double>()) );
    ui->min_distance_slide->setMaximum(static_cast<int>(10 * flt["min_distance_threshold"].get<double>()) );
    ui->floor_gap_slide->setMaximum(static_cast<int>(10 * flt["max_floor_gap"].get<double>()) );
    ui->ceil_gap_slide->setValue(static_cast<int>(10 * flt["ceil_gap"].get<double>() ) );
    ui->ceil_gap_slide->setMaximum(static_cast<int>(10 * flt["max_ceil_gap"].get<double>() ) );

    ui->angle_start_text->setText( QString::number(ui->angle_start_slide->value()) );
    ui->angle_size_text->setText( QString::number(ui->angle_size_slide->value()) );
    ui->max_distance_text->setText( QString::number( flt["max_distance"].get<double>() )  );
    ui->min_distance_text->setText( QString::number( flt["min_distance"].get<double>() )  );
    ui->floor_gap_text->setText( QString::number( flt["floor_gap"].get<double>() )  );
    ui->ceil_gap_text->setText( QString::number( flt["ceil_gap"].get<double>() )  );


    connect(ui->angle_start_slide, &QSlider::valueChanged, this, &MainWindow::processSlider);
    connect(ui->angle_size_slide, &QSlider::valueChanged, this, &MainWindow::processSlider);
    connect(ui->max_distance_slide, &QSlider::valueChanged, this, &MainWindow::processSlider);
    connect(ui->min_distance_slide, &QSlider::valueChanged, this, &MainWindow::processSlider);
    connect(ui->floor_gap_slide, &QSlider::valueChanged, this, &MainWindow::processSlider);
    connect(ui->ceil_gap_slide, &QSlider::valueChanged, this, &MainWindow::processSlider);
}

void MainWindow::openDataset(const QString &dataset_path)
{
    if( dataset_path.isEmpty() ) { cout << "Dataset path is empty!" << endl; return;}

    data_reader_.reset(new DataReader(dataset_path));
    if(!data_reader_->isValid())
    {
        data_reader_ = nullptr;
        QMessageBox::warning(this, tr("Warn"), tr("The directory is invalid"));
        return;
    }

    auto& JK = js_["cam"]["K"];
    auto& JD = js_["cam"]["D"];
    auto& JTag = js_["tag_size"];
    for(uint8_t i=0; i<9; i++)
    {
        K_(i / 3, i % 3) = JK[i / 3][i % 3];
        if(i<5)
        {
            D_(i) = JD[i];
        }
        if(i<3){
            tag_size_(i) = JTag[i];
        }
    }
    data_reader_->setCameraK(K_);
    data_reader_->setCameraD(D_);
    data_reader_->setTagSize(tag_size_);
    data_root_ = dataset_path;
    ui->total_data_num->setText(QString::number(data_reader_->getDatasetSize()));

    img_viewer_.reset(new ImageViewer);
    img_viewer_->show();
    std::vector<double> cameraPerspectiveParams;
    auto PParams = js_["pc"]["PerspectiveParams"];
    for(uint8_t i = 0; i < 6; i++)
    {
        cameraPerspectiveParams.push_back(PParams[i]);
    }
    pc_viewer_.reset( new PointcloudViewer(cameraPerspectiveParams) );
    pc_viewer_->show();

    processData();
}

void MainWindow::updateLabels()
{
    if(sensor_data_.size() > 0)
    {
        ui->current_data_id->setText(QString::number(sid_));

        uint32_t num  = uint32_t (sensor_data_.size() );
        if(!sensor_data_.back().good())
        {
            num -= 1;
        }
        ui->processed_data_num->setText(QString::number(num));
    }
    else
    {
        ui->current_data_id->setText("Null");
        ui->processed_data_num->setText("0");
    }
}

/**
 * @brief 將新来的一帧数据加入sensor_data_中，并预处理，open dataset和next pose时会调用
 * 
 * @param is_check 
 * @return true 
 * @return false 
 */
bool MainWindow::processData(bool is_check)
{
    if(data_reader_ == nullptr)
    {
        QMessageBox::warning(this, tr("Error"), tr("Please open dataset first!"));
        return false;
    }

    if( is_check && sensor_data_.size()>0 && !sensor_data_.back().good() && !skip_ids_.count(sid_))  // ensure that last frame is normal if last frame is not skipped.
    {
        QMessageBox::warning(this, tr("Error"), tr("Current data is not good, adjust it!"));
        return false;
    }

    if(sensor_data_.size() > 0 )
    {
        if( !data_reader_->moveNext() )
        {
            ui->next_pose->setEnabled(false);
            QMessageBox::information(this, tr("Success"), tr("Feature extraction completed!\nPlease click  Calibrate!"));
            return false;
        }
    }

    sensor_data_.emplace_back(data_reader_->getCurrentId());// add a new data frame to sensor_data_
    sid_ = uint32_t( sensor_data_.size()-1 );
    auto& last = sensor_data_.back();
    last.img = data_reader_->getImage();
    if(!last.img )
    {
        sensor_data_.pop_back();
        QMessageBox::warning(this, tr("Error"), tr("Fail to read image"));
        return false;
    }

    last.pc = data_reader_->getPointcloud();
    if(!last.pc)
    {
        sensor_data_.pop_back();
        QMessageBox::warning(this, tr("Error"), tr("Fail to read pointcloud!"));
        return false;
    }
    // filter
    if(last.pc->size() > 1000000) { // voxel grid downsample if the pc has very much points;  
        pcl::VoxelGrid<pcl::PointXYZI> vg_filter;
        vg_filter.setLeafSize(0.02, 0.02, 0.02);
        vg_filter.setInputCloud(last.pc);
        vg_filter.filter(*last.pc);
    }
    const double MIN_RADIUS = 1, MAX_RADIUS = 18;
    {
        auto tmp_pc = *last.pc;
        last.pc->clear(); 
        for(const auto& p : tmp_pc.points) {
            double curr_dist = p.x * p.x + p.y * p.y + p.z * p.z;
            if(curr_dist <= MIN_RADIUS * MIN_RADIUS 
                || curr_dist >= MAX_RADIUS * MAX_RADIUS) {
                continue;
            }
            last.pc->push_back(p);
        }
    }

    last.img_marked = std::make_shared<cv::Mat>();
    last.img->copyTo(*last.img_marked);
    last.pc_marked.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    last.pc_plane.reset(new pcl::PointCloud<pcl::PointXYZI>);
    auto& ply = last.polygon;
    //img
    std::vector<cv::Point2d> img_corners(4); 
    double error_fraction = 0.5;
    getCorners(*last.img,img_corners,error_fraction);   // extract image corners by detecting AprilTag
    ply.img = std::make_shared<ImagePolygon>(4);
    if(img_corners[0].x != 0 && img_corners[0].y != 0) // 检测到apriltag 
    {
        auto compare_rule = [](cv::Point2d p1, cv::Point2d p2){
            return p1.y < p2.y;// ascending 
        };
        std::sort(img_corners.begin(), img_corners.end(), compare_rule);
        std::swap(img_corners[2],img_corners[3]);// ensure the sequence
        ply.img->InitPolygon(img_corners);
        ply.img->MarkImage(*last.img , *last.img_marked);
        last.img_good = true;
    }
    //pc
    ply.pc  = pcply_->Add(last.pc, last.pc_marked , last.pc_plane,last.pc_plane_coef);
    last.pid = uint32_t (sensor_data_.size()-1);
    if(ply.pc != nullptr)
    {
        last.pc_valid = true;
    }


    img_viewer_->showImage(last.img_marked);
    pc_viewer_->showPointcloud(last.pc_marked , last.pc_plane);

    sid_ = uint32_t( sensor_data_.size()-1 );
    updateLabels();

    return true;
}

void MainWindow::setEnabledAll(bool status)
{
    QList<QPushButton*> btns = ui->centralWidget->findChildren<QPushButton*>();
    for(auto& it : btns)
    {
        it->setEnabled(status);
    }

    QList<QSlider*> sliders = ui->pointcloud_group->findChildren<QSlider*>();
    for(auto& it : sliders)
    {
        it->setEnabled(status);
    }
}

void MainWindow::project(std::shared_ptr<cv::Mat> img_orig, std::shared_ptr<cv::Mat> img_proj, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_proj_cam)
{
    assert( img_orig != nullptr && img_proj != nullptr && pc_proj_cam != nullptr );

    const double resolution = 0.1;
    const double depth_min = 1.0;
    const double depth_gap = 10.0;

    uint32_t img_width = img_proj->cols;
    uint32_t img_height = img_proj->rows;

    uint32_t  num = static_cast<uint32_t>(depth_gap/resolution);
    Color::rgbs colors = Color::get_rgbs(num);
    std::unordered_set<int> used_img_idx;

    int occlusion_size = 0;
    for(auto& p : pc_proj_cam->points)
    {
        Eigen::Vector3d pt;

        pt = K_ * Eigen::Vector3d(p.x, p.y, p.z);

        if(pt(2) < 0.2)
        {
            continue;
        }
        int32_t u = static_cast<int32_t>(pt(0)/pt(2));
        int32_t v = static_cast<int32_t>(pt(1)/pt(2));

        if(u < 0 || u >= img_proj->cols || v<0 || v >= img_proj->rows)
        {
            continue;
        }
        int proj_pixel_id = img_width * v + u;
        if(used_img_idx.find(proj_pixel_id) != used_img_idx.end()) {
            p.r = 0xff;
            p.g = 0;
            p.b = 0;
            p.a = 0;
            p.x = 0;
            p.y = 0;
            p.z = 0;
            occlusion_size++;
            continue;
        }
        used_img_idx.insert(proj_pixel_id);
        cv::Vec3b color = img_orig->at<cv::Vec3b>(cv::Point(u,v));
        p.r = color[2];
        p.g = color[1];
        p.b = color[0];
        double f = std::sqrt(p.x*p.x + p.z*p.z) - depth_min;
        uint32_t idx = static_cast<uint32_t>(f/resolution);
        if(idx >= num)
        {
            idx = num -1;
        }
        auto& c = colors[idx];
        double marker_redius = img_width/480 ;
        // img_proj->at<cv::Vec3b>(v, u) = cv::Vec3b(c[0], c[1], c[2]);
        cv::circle(*img_proj, cv::Point2d(u,v), marker_redius, cv::Scalar(c[0], c[1], c[2]), -1);
    }
    // cout << occlusion_size << endl;
}

void MainWindow::showCalibrateResult()
{
    auto& sd = sensor_data_[sid_];

    if(sd.img_proj == nullptr || sd.pc_proj == nullptr || sd.pc_proj_cam == nullptr)
    {
        sd.img_proj.reset(new cv::Mat);
        sd.img->copyTo(*sd.img_proj);
        sd.pc_proj.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        sd.pc_proj_cam.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*sd.pc, *sd.pc_proj);
        for(auto& p: sd.pc_proj->points)
        {
            p.rgba = 0xffffffff;
        }
        pcl::transformPointCloud(*sd.pc_proj, *sd.pc_proj_cam, T_.cast<float>());
        sort(sd.pc_proj_cam->points.begin(), sd.pc_proj_cam->points.end(), [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2){
            return (p1.x*p1.x + p1.y*p1.y + p1.z*p1.z) < (p2.x*p2.x + p2.y*p2.y + p2.z*p2.z) ;
        });
    }
    project(sd.img, sd.img_proj, sd.pc_proj_cam);
    pcl::transformPointCloud(*sd.pc_proj_cam, *sd.pc_proj, T_.inverse().cast<float>());

    img_viewer_->showImage(sd.img_proj);
    pc_viewer_->showPointcloud(sd.pc_proj , sd.pc_plane);

}

void MainWindow::processSlider()
{
    std::ofstream f(config_path_.toStdString());
    if(!f.good())
    {
        QMessageBox::warning(this, tr("Error"), tr("Fail to open config file"));
        return;
    }

    Eigen::VectorXd param;
    param.resize(6);
    param(0) = ui->angle_start_slide->value();
    param(1) = ui->angle_size_slide->value();
    param(2) = ui->max_distance_slide->value()/10.0;
    param(3) = ui->floor_gap_slide->value()/10.0;
    param(4) = ui->ceil_gap_slide->value()/10.0;
    param(5) = ui->min_distance_slide->value()/10.0;

    ui->angle_start_text->setText( QString::number(param(0)) );
    ui->angle_size_text->setText( QString::number(param(1)) );
    ui->max_distance_text->setText( QString::number(param(2)) );
    ui->floor_gap_text->setText( QString::number(param(3)) );
    ui->ceil_gap_text->setText( QString::number(param(4)) );
    ui->min_distance_text->setText( QString::number(param(5)) );

    if(sensor_data_.size() == 0 )
    {
        return;
    }
    auto& filter = js_["pc"]["filter"];
    filter["angle_start"] = param(0);
    filter["angle_size"] = param(1);
    filter["max_distance"] = param(2);
    filter["min_distance"] = param(5);
    filter["floor_gap"] = param(3);
    filter["ceil_gap"] = param(4);
    f << js_.dump(4);
    f.close();
    auto& sd = sensor_data_.back();

    auto& ply = sd.polygon;
    pcply_->SetFilterParameters(param);
    ply.pc = pcply_->Add(sd.pc, sd.pc_marked , sd.pc_plane,sd.pc_plane_coef);

    if(ply.pc != nullptr)
    {
        sd.pc_valid = true;
    }

    // visualize the filter point cloud and the extracted plane 
    pc_viewer_->showPointcloud(sd.pc_marked ,sd.pc_plane );
}


