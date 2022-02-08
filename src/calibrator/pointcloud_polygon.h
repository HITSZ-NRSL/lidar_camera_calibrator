/**
  ******************************************************************************
  * @file	pointcloud_polygon.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-26
  * @brief	pointcloud_polygon.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POINTCLOUD_POLYGON_H
#define __POINTCLOUD_POLYGON_H

/* Includes ------------------------------------------------------------------*/
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

#include "utils.h"
#include "json/json.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
//namespace lqh
//{
class PointcloudPolygon
{
public:
    using PCI = pcl::PointCloud<pcl::PointXYZI>;
    using Indices = std::vector<int>;
    size_t plane_pc_num;
	struct Polygon3D
	{
		using ConstPtr = std::shared_ptr<const Polygon3D>;
		using Ptr = std::shared_ptr<Polygon3D>;

        Eigen::Vector4d 	coef;// coefficent of the plane
        std::vector<int> indices;

        Polygon3D(uint32_t size)
        {

        }

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	explicit PointcloudPolygon(const nlohmann::json& js, uint8_t size);
    Polygon3D::ConstPtr Add(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcc ,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_plane,
                            Eigen::Vector4d& coef);
    void GetPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc,
                  const Indices& indices ,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_plane);
    void SetFilterParameters(const Eigen::VectorXd& p);

    /**
     * @brief Get the Filter Parameters object( Unused )
     *  
     * @return Eigen::Vector4d 
     */
    Eigen::Vector4d GetFilterParameters()
    {
        return Eigen::Vector4d(filter_angle_start_, filter_angle_size_,
                               filter_max_distance_, filter_floor_gap_);
    }
    bool SaveMarkedPointcloud(const std::string& fn, const PCI& pc,
                              const Indices& indices) const;
private:


	const uint8_t 	size_;
	double 		filter_angle_start_;
	double 		filter_angle_size_;
	double 		filter_min_distance_;
	double 		filter_max_distance_;
	double 		filter_floor_gap_;
	double 		filter_ceil_gap_;
	uint32_t 	plane_point_num_min_;
	double 		plane_distance_threshold_;

	Polygon3D::Ptr polygon_;

	void SectorFilter(const PCI& pc, Indices& indices);
	bool ExtractPlane(const PCI& pc, Indices& indices, Eigen::Vector4d& coef);
	Polygon3D::Ptr ExtractEdgeInlier(const PCI& pc, const Indices& indices);
	void FitLine(const PCI& pc, const Indices& indices,
				 Indices& inlier, Eigen::Vector3d& coef, Eigen::Vector3d& pt);

    void MarkPointcloud(pcl::PointCloud<pcl::PointXYZRGB>& pcc,
                        const lqh::utils::color::rgb& color,
                        const Indices& indices) const;

};

//}

/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* !__POINTCLOUD_POLYGON_H */

/*****************************END OF FILE**************************************/
