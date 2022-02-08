/**
  ******************************************************************************
  * @file	image_polygon.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-22
  * @brief	image_polygon.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMAGE_POLYGON_H
#define __IMAGE_POLYGON_H

/* Includes ------------------------------------------------------------------*/

#include <string>
#include <vector>
#include <memory>
#include <stdint.h>

#include <Eigen/Dense>

#include "json/json.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//namespace lqh
//{
class ImagePolygon
{
public:


    using ConstPtr = std::shared_ptr<const ImagePolygon>;
    using Ptr = std::shared_ptr<ImagePolygon>;

    Eigen::Vector3d center;
    Eigen::Matrix3Xd vertexs;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ImagePolygon(uint8_t size);
    void InitPolygon(const cv::Point2d pts[]);
    void InitPolygon(const std::vector<cv::Point2d>& pts);
    void MarkImage(const cv::Mat& img, cv::Mat& img_out);
private:
	const uint32_t size_;	// current implemention only support size_=4

};

//}

/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* !__IMAGE_POLYGON_H */

/*****************************END OF FILE**************************************/
