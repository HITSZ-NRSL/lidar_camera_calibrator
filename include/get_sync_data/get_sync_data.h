/**
 * @file get_sync_data.h
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-05
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <thread>
#include <mutex>
#include <algorithm>

#include<sys/stat.h>    
#include <unistd.h>
#include <cstdlib>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>   //allows us to use pcl::transformPointCloud function

class GetSyncData{
public:
    using pcMsg = sensor_msgs::PointCloud2;
    using imgMsg = sensor_msgs::Image;
    using syncPolicy = message_filters::sync_policies::ApproximateTime<pcMsg, imgMsg>;
    using pcMFSub = message_filters::Subscriber<pcMsg> ;
    using pcMFSubPtr = boost::shared_ptr<pcMFSub> ;
    using imgMFSub = message_filters::Subscriber<imgMsg> ;
    using imgMFSubPtr = boost::shared_ptr<imgMFSub> ;
    using MFSync = message_filters::Synchronizer<syncPolicy>;
    using MFSyncPtr = boost::shared_ptr<MFSync>;
    GetSyncData(ros::NodeHandle n);

private:
    void readROSParameters();

    void sync_callback(pcMsg::ConstPtr pc, imgMsg::ConstPtr img);

    void save_img(imgMsg::ConstPtr img, const std::string path);
    void save_pc(pcMsg::ConstPtr pc, const std::string path);

    void getFlag(){
        /*
        f: save current synchronized data
        */
        while(ros::ok()){
            // mu_.lock();
            flag = getch();
            // std::cout << "flag: " << flag << std::endl;
            // mu_.unlock();
            usleep(100);
        }
    }

    char getch()
    {
        struct termios oldattr, newattr;
        int ch;
        tcgetattr( STDIN_FILENO, &oldattr );
        newattr = oldattr;
        newattr.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
        ch = getchar();
        tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
        return ch;
    }

    // ros
    ros::NodeHandle n_;
    pcMFSubPtr pc_mf_sub_;
    imgMFSubPtr img_mf_sub_;
    MFSyncPtr synchronizer_;

    // thread 
    char flag;
    std::mutex mu_;  
    boost::shared_ptr<std::thread> thread_get_char_;

    // param
    std::string img_topic_;
    std::string pc_topic_;
    double freq_;
    std::string output_path_;

    std::string img_output_path_;
    std::string pc_output_path_;
    int idx;
};