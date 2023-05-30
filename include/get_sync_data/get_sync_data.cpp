/**
 * @file get_sync_data.cpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-05
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include "get_sync_data/get_sync_data.h"

namespace fs = boost::filesystem;

GetSyncData::GetSyncData(ros::NodeHandle n):
n_(n),
img_topic_("/camera/color/image_raw"),
pc_topic_("/rslidar_points"),
freq_(10),
idx(0)
{
    readROSParameters();

    // initialization
    if (!fs::exists(img_output_path_)) {
        if (!fs::create_directories(img_output_path_)) {
            ROS_ERROR("Can't make dir: %s", img_output_path_.c_str());
            ros::shutdown();
        }
    } else {
        if (count_files(img_output_path_) > 0) {
            ROS_ERROR("img output path contains files! Will not overwrite"); 
            ros::shutdown();   
        }
    }

    if (!fs::exists(pc_output_path_)) {
        if (!fs::create_directories(pc_output_path_)) {
            ROS_ERROR("Can't make dir: %s", pc_output_path_.c_str());
            ros::shutdown();
        }
    } else {
        if (count_files(pc_output_path_) > 0) {
            ROS_ERROR("pc output path contains files! Will not overwrite"); 
            ros::shutdown();   
        }
    }

    thread_get_char_ = boost::shared_ptr<std::thread>( new std::thread( &GetSyncData::getFlag, this) );
    pc_mf_sub_ = pcMFSubPtr( new pcMFSub(n_, pc_topic_, 1) );
    img_mf_sub_ = imgMFSubPtr( new imgMFSub(n_, img_topic_, 1) );
    synchronizer_ = MFSyncPtr( new MFSync(syncPolicy(freq_), *pc_mf_sub_, *img_mf_sub_) );
    synchronizer_->registerCallback(boost::bind(&GetSyncData::sync_callback, this, _1, _2));
}

void GetSyncData::readROSParameters(){
    if(ros::param::get("img_topic",img_topic_)) {
        ROS_INFO("Parameters: img_topic(%s)", img_topic_.c_str());
    }
    if(ros::param::get("pc_topic",pc_topic_)) {
        ROS_INFO("Parameters: pc_topic(%s)", pc_topic_.c_str());
    }
    if(ros::param::get("frequency",freq_)) {
        ROS_INFO("Parameters: frequency(%f)", freq_);
    }

    std::string path_str;
    if(ros::param::get("output_path", path_str)) {
        ROS_INFO("Parameters: output_path(%s)", path_str.c_str());
        output_path_ = fs::path(path_str);
    } else {
        ROS_ERROR("Parameters: output_path is not defined!");
        ROS_BREAK();
    }

    img_output_path_ = output_path_ / "image_orig";
    ROS_INFO("Parameters: img_output_path(%s)", img_output_path_.c_str());

    pc_output_path_ = output_path_ / "pointcloud";
    ROS_INFO("Parameters: pc_output_path(%s)", pc_output_path_.c_str());

    ROS_INFO("Ready. Press 'space' to capture frame");
}

size_t GetSyncData::count_files(const fs::path& dir) const {
    size_t count = std::count_if(
        fs::directory_iterator(dir),
        fs::directory_iterator(),
        [](auto& entry) { return fs::is_regular_file(entry); });

    return count;
}

void GetSyncData::sync_callback(pcMsg::ConstPtr pc, imgMsg::ConstPtr img) {
    switch (flag)
    {
    case ' ':
        {
            ROS_INFO("Get:\npc:\t%f\nimg:\t%f", pc->header.stamp.toSec(), img->header.stamp.toSec());

            auto idx_str = std::to_string(idx);
            fs::path img_path = img_output_path_ / idx_str;
            img_path.replace_extension(".jpg"); 
            save_img(img, img_path.string());

            fs::path pc_path = pc_output_path_ / idx_str;
            pc_path.replace_extension(".pcd");
            save_pc(pc, pc_path.string());
            flag = '\0';

            idx++;
            break;
        }
    default:
        {
            break;
        }
    }

}

void GetSyncData::save_img(imgMsg::ConstPtr img, const std::string path) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::imwrite(path, cv_ptr->image);
}

void GetSyncData::save_pc(pcMsg::ConstPtr pc, const std::string path) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    pcl::io::savePCDFileASCII(path, *temp_cloud);
}
