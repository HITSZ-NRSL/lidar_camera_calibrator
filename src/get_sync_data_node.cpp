/**
 * @file get_sync_data_node.cpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-05
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include "get_sync_data/get_sync_data.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_sync_data");
    ros::NodeHandle n;
    GetSyncData get_sync_data(n);
    ros::spin();

    return 0;
}
