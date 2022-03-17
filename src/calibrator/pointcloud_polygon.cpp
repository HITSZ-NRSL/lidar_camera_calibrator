/**
  ******************************************************************************
  * @file	pointcloud_polygon.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-26
  * @brief	pointcloud_polygon.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pointcloud_polygon.h"

#include <cmath>
#include <map>
#include <functional>

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace lqh;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//#define NODEBUG

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief
  * @param
  * @note
  * @return None
  */
PointcloudPolygon::PointcloudPolygon(const nlohmann::json& js, uint8_t size):
    plane_pc_num(0),size_(size), polygon_(nullptr)
{
	filter_angle_start_ = js["filter"]["angle_start"].get<double>();
	filter_angle_size_ 	= js["filter"]["angle_size"].get<double>();
	filter_max_distance_  	= js["filter"]["max_distance"].get<double>();
	filter_min_distance_  	= js["filter"]["min_distance"].get<double>();
	filter_floor_gap_ 	= js["filter"]["floor_gap"].get<double>();
	filter_ceil_gap_ 	= js["filter"]["ceil_gap"].get<double>();
	plane_point_num_min_ 		= js["plane"]["point_num_min"].get<uint32_t>();
	plane_distance_threshold_ 	= js["plane"]["distance_threshold"].get<double>();
}

/**
 * @brief 输入当前帧点云，对当前帧点云进行平面提取
 * 
 * @param pc 输入
 * @param pcc 将filter后的点云上色后得到的彩色点云
 * @param pc_plane 对filter后的点云提取平面的结果
 * @param coef 提取平面的平面方程
 * @return PointcloudPolygon::Polygon3D::ConstPtr 
 */
PointcloudPolygon::Polygon3D::ConstPtr PointcloudPolygon::Add(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcc ,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_plane,
        Eigen::Vector4d& coef)
{
	Polygon3D::Ptr ply = std::make_shared<Polygon3D>(size_);
	std::vector<int> indices;
    if(pcc->size() != pc->size())
	{
        pcl::copyPointCloud(*pc, *pcc);
	}
    for(auto& p : pcc->points)
	{
		p.rgba = 0xffffffff;
	}


	lqh::utils::color::rgbs colors = lqh::utils::color::get_rgbs(5);

	ply->coef.setZero();

    SectorFilter(*pc, indices);
    MarkPointcloud(*pcc, colors[0], indices);
//	if(indices.size() < plane_point_num_min_)
//	{
//		return nullptr;
//	}

//	SaveMarkedPointcloud("SectorFilter.pcd", pc, indices);

    if(plane_pc_num != indices.size())
    {
        plane_pc_num = indices.size();
        std::cout<< "\033[1;32mPointCloudExtract\033[0m:\t" << "current plane points number : "<<plane_pc_num<<std::endl;
    }

    if(indices.size()){
        ExtractPlane(*pc, indices, ply->coef);
        MarkPointcloud(*pcc, colors[1], indices);
        ply->indices = indices;
        polygon_ = ply;
        coef = ply->coef;
        GetPlane(pc, indices ,pc_plane);
    }
    else {
        pc_plane->clear();
        coef.setZero();
    }
	return ply;
}
void PointcloudPolygon::GetPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc,
                                 const Indices& indices ,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_plane)
{
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;


    pcl::IndicesPtr indices_ptr(new std::vector<int>);
    indices_ptr->resize(indices.size());
    std::copy(indices.begin(), indices.end(), indices_ptr->begin());
    extract.setInputCloud (pc);
    extract.setIndices (indices_ptr);
    extract.setNegative (false);
    extract.filter (*pc_plane);

}
void PointcloudPolygon::SetFilterParameters(const Eigen::VectorXd& p)
{
    if(p(0) >=0 && p(0) <=360)
    {
        filter_angle_start_ = p(0);
    }
    if(p(1) >0 && p(1) < 360)
    {
        filter_angle_size_  = p(1);
    }
    if(p(2) > 0)
    {
        filter_max_distance_    = p(2);
    }
    if(p(3) > -10)
    {
        filter_floor_gap_   = p(3);
    }
    filter_ceil_gap_   = p(4);
    filter_min_distance_ = p(5);
}

void PointcloudPolygon::SectorFilter(const PCI& pc, Indices& indices)
{
	indices.clear();
    indices.reserve(plane_point_num_min_*10);

	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(pc, min_pt, max_pt);

	double z_min_threshold = min_pt(2) + filter_floor_gap_;
	double z_max_threshold = max_pt(2) - filter_ceil_gap_;

	for(uint32_t i=0; i<pc.points.size(); i++)
	{
		auto& p = pc.points[i];
		double theta = std::atan2(p.y, p.x)/MATH_PI*180;
		if(theta<0)
		{
			theta += 360;
		}
		double dis = std::sqrt(p.y*p.y + p.x*p.x);
		double size = theta - filter_angle_start_;
		if(size < 0)
		{
			size += 360;
		}

        if(p.z > z_min_threshold && p.z < z_max_threshold && dis < filter_max_distance_
			&& dis > filter_min_distance_ && size < filter_angle_size_)
		{
			indices.push_back(i);
		}
	}
}

bool PointcloudPolygon::ExtractPlane(const PCI& pc, Indices& indices,
									 Eigen::Vector4d& coef)
{// extract point cloud's polygon
	PCI::Ptr pc_ptr (new PCI);
    pcl::copyPointCloud(pc, *pc_ptr);

	pcl::IndicesPtr indices_ptr(new std::vector<int>);
	indices_ptr->resize(indices.size());
    std::copy(indices.begin(), indices.end(), indices_ptr->begin());

    pcl::ModelCoefficients model_coef;
	pcl::PointIndices ids;
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (plane_distance_threshold_);
	seg.setInputCloud(pc_ptr);
    seg.setIndices(indices_ptr);
	seg.segment(ids, model_coef);

    if(ids.indices.size() != 0 )
	{
		coef << model_coef.values[0], model_coef.values[1],
			 model_coef.values[2], model_coef.values[3];
		std::swap(ids.indices, indices);
		return true;
	}
}

void PointcloudPolygon::MarkPointcloud(pcl::PointCloud<pcl::PointXYZRGB>& pcc,
									   const lqh::utils::color::rgb& color,
									   const Indices& indices) const
{
	for(auto it : indices)
	{
		auto& p = pcc.points[it];
		p.r = color[0];
		p.g = color[1];
		p.b = color[2];
	}
}
bool PointcloudPolygon::SaveMarkedPointcloud(const std::string& fn, const PCI& pc,
        const Indices& indices) const
{
    pcl::PointCloud<pcl::PointXYZRGB> pcc;
    pcl::copyPointCloud(pc, pcc);

    for(auto& it:pcc.points)
    {
        it.rgba = 0xffffffff;
    }

    for(auto it: indices)
    {
        pcc.points[it].rgba = 0xffff0000;
    }

    return pcl::io::savePCDFileBinary(fn, pcc);
}

/*****************************END OF FILE**************************************/
