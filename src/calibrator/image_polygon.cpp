/**
  ******************************************************************************
  * @file	image_polygon.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-22
  * @brief	image_polygon.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "image_polygon.h"

#include <iostream>
#include <cmath>

#include "utils.h"

using namespace lqh;

/**
  * @brief
  * @param
  * @note
  * @return None
  */
ImagePolygon::ImagePolygon( uint8_t size):
    size_(size)
{
    center.setZero();
    vertexs.resize(3, size);
}
void ImagePolygon::InitPolygon(const cv::Point2d pts[])
{
    std::vector<cv::Point2d> pts_vector;
    for(int i = 0; i < 4 ; i ++)
    {
        pts_vector.push_back(pts[i]);
    }
    InitPolygon(pts_vector);
}
void ImagePolygon::InitPolygon(const std::vector<cv::Point2d>& pts)
{
    using PairUD = std::pair<uint32_t, double>;
    Eigen::Matrix3Xd points(3 , size_) ;
    points.row(2).setOnes();
    for(uint32_t i=0; i<size_; i++)
    {
        points( 0 , i) = pts[i].x;
        points( 1 , i) = pts[i].y;
    }
    // sort the points(clock wise)
//    std::vector<PairUD> verts( size_ );

//    for(uint32_t i=0; i<size_; i++)
//    {
//        verts[i].first = i;
//        verts[i].second = std::atan2(points(1,i)-center(1), points(0,i)-center(0)) + MATH_PI;
//    }
//    std::sort(verts.begin(), verts.end(), [](PairUD& a, PairUD& b)
//    {
//        return a.second < b.second;
//    });
//    for(uint32_t i=0; i<size_; i++)
//    {
//    vertexs.col(i) = points.col(verts[i].first);
//    }
    vertexs = points;
    // center
    center = points.rowwise().mean();
    center(2) = 1;
}

/**
 * @brief merge all paralle lines
 * @param ls
 * @return
 */

//@TODO only apply to rectangular board
void ImagePolygon::MarkImage(const cv::Mat& img, cv::Mat& img_out)
{
    img.copyTo(img_out);

    utils::color::rgbs colors = utils::color::get_rgbs( size_ );
    for (uint32_t i = 0; i < size_; i++)
    {
        uint32_t next = (i== size_-1)?0:i+1;
        auto& it = vertexs;

        cv::Scalar color(colors[i][2], colors[i][1], colors[i][0]);
        cv::Point2d p0( it(0,i), it(1,i) );
        cv::Point2d p1( it(0,next), it(1, next) );

        // draw vertex
        cv::circle(img_out, p0, double(img_out.rows) /150 , cv::Scalar(255,0,0),img_out.rows/150 + 3);
        // draw line
        cv::line(img_out,p0, p1, cv::Scalar(0,0,255) , img_out.rows/200);
        cv::putText(img_out, std::to_string(i+1),p0,cv::FONT_HERSHEY_SIMPLEX,1.2,CV_RGB(0,255,0),2.2);
    }
}
/*****************************END OF FILE**************************************/
