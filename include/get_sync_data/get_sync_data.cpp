/**
 * @file get_sync_data.cpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-05
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include "get_sync_data/get_sync_data.h"

using namespace std;

GetSyncData::GetSyncData(ros::NodeHandle n):
n_(n),
img_topic_("/camera/color/image_raw"),
pc_topic_("/rslidar_points"),
freq_(10),
idx(0)
{
    readROSParameters();

    // initialization
    if(access( img_output_path_.c_str(), 0) == -1 ) {
        string command = "mkdir -p " + img_output_path_;
        system(command.c_str());
        if(access( img_output_path_.c_str(), 0) == -1 ) {
            ROS_ERROR("Can't make dir: %s", img_output_path_.c_str());
            ros::shutdown();
        }
    }
    if(access( pc_output_path_.c_str(), 0) == -1 ) {
        string command = "mkdir -p " + pc_output_path_;
        system(command.c_str());
        if(access( pc_output_path_.c_str(), 0) == -1 ) {
            ROS_ERROR("Can't make dir: %s", pc_output_path_.c_str());
            ros::shutdown();
        }
    }

    thread_get_char_ = boost::shared_ptr<thread>( new thread( &GetSyncData::getFlag, this) );
    pc_mf_sub_ = pcMFSubPtr( new pcMFSub(n_, pc_topic_, 1) );
    img_mf_sub_ = imgMFSubPtr( new imgMFSub(n_, img_topic_, 1) );
    synchronizer_ = MFSyncPtr( new MFSync(syncPolicy(freq_), *pc_mf_sub_, *img_mf_sub_) );
    synchronizer_->registerCallback(boost::bind(&GetSyncData::sync_callback, this, _1, _2));
}

void GetSyncData::readROSParameters(){
    if(ros::param::get("img_topic",img_topic_)){
        ROS_INFO("Parametes: img_topic(%s)", img_topic_.c_str());
    }
    if(ros::param::get("pc_topic",pc_topic_)){
        ROS_INFO("Parametes: pc_topic(%s)", pc_topic_.c_str());
    }
    if(ros::param::get("frequency",freq_)){
        ROS_INFO("Parametes: frequency(%f)", freq_);
    }
    if(ros::param::get("output_path",output_path_)){
        ROS_INFO("Parametes: output_path(%s)", output_path_.c_str());
    } else {
        ROS_ERROR("Parametes: output_path is not defined!");
        ROS_BREAK();
    }
    if(output_path_.back() != '/') output_path_.push_back('/');
    img_output_path_ = output_path_ + "image_orig/";
    pc_output_path_ = output_path_ + "pointcloud/";
}

void GetSyncData::sync_callback(pcMsg::ConstPtr pc, imgMsg::ConstPtr img){
    // lock_guard<mutex> mu_guard(mu_);
    switch (flag)
    {
    case ' ':
        {
            ROS_INFO("Get:\npc:\t%f\nimg:\t%f", pc->header.stamp.toSec(), img->header.stamp.toSec());
            save_img(img, img_output_path_ + to_string(idx) + ".jpg");
            save_pc(pc, pc_output_path_ + to_string(idx++) + ".pcd");
            flag = '\0';
            break;
        }
    
    default:
        {
            // ROS_INFO("Nothing: \t%f", pc->header.stamp.toSec());
            break;
        }
    }

}

void GetSyncData::save_img(imgMsg::ConstPtr img, const std::string path){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::imwrite(path, cv_ptr->image);
}

void GetSyncData::save_pc(pcMsg::ConstPtr pc, const std::string path){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    pcl::io::savePCDFileASCII(path, *temp_cloud);
}