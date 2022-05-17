/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 ********************************************************************/

#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "TagDetector.h"
#include "DebugImage.h"

#define DEFAULT_TAG_FAMILY "Tag36h11"

void getInitCorners(const cv::Mat& im , cv::Point2d corners[])
{   
    // up , down
    for(int i = 0; i < im.rows - 1 ; i++)
    {
        cv::Scalar current_mean_temp = cv::mean(im.rowRange(i,i+1));
        cv::Scalar next_mean_temp = cv::mean(im.rowRange(i+1,i+2));
        double current_sum(0) , next_sum(0);
        for(int j = 0; j < 4 ;j++)
        {
            current_sum += current_mean_temp.val[j];
            next_sum += next_mean_temp.val[j];
        }
        //up
        if(current_sum == 0 && next_sum > 0)
        {
            int j;
            for(j = 0; j < im.cols ;j ++)
            {
                if( im.at<cv::Vec3b>(i+1,j)[0] ||
                    im.at<cv::Vec3b>(i+1,j)[1] ||
                    im.at<cv::Vec3b>(i+1,j)[2])
                {
                    break;
                }
            }
            corners[0] = cv::Point2d(j , i+1) ;
        }
        //down
        if(current_sum > 0 && next_sum == 0)
        {
            int j;
            for(j = 0; j < im.cols ;j ++)
            {
                if( im.at<cv::Vec3b>(i,j)[0] ||
                    im.at<cv::Vec3b>(i,j)[1] ||
                    im.at<cv::Vec3b>(i,j)[2])
                {
                    break;
                }
            }
            corners[2] = cv::Point2d( j , i) ;
        }
    }
    // left right
    for(int i = 0; i < im.cols - 1 ; i++)
    {
        cv::Scalar current_mean_temp = cv::mean(im.colRange(i,i+1));
        cv::Scalar next_mean_temp = cv::mean(im.colRange(i+1,i+2));
        double current_sum(0) , next_sum(0);
        for(int j = 0; j < 4 ;j++)
        {
            current_sum += current_mean_temp.val[j];
            next_sum += next_mean_temp.val[j];
        }
        //left
        if(current_sum == 0 && next_sum > 0)
        {
            int j;
            for(j = 0; j < im.rows ;j ++)
            {
                if( im.at<cv::Vec3b>(j,i+1)[0] ||
                    im.at<cv::Vec3b>(j,i+1)[1] ||
                    im.at<cv::Vec3b>(j,i+1)[2])
                {
                    break;
                }
            }
            corners[3] = cv::Point2d( i+1,j ) ;
        }
        //down
        if(current_sum > 0 && next_sum == 0)
        {
            int j;
            for(j = 0; j < im.cols ;j ++)
            {
                if( im.at<cv::Vec3b>(j,i)[0] ||
                    im.at<cv::Vec3b>(j,i)[1] ||
                    im.at<cv::Vec3b>(j,i)[2])
                {
                    break;
                }
            }
            corners[1] = cv::Point2d(i , j) ;
        }
    }

}

bool compareCorners(const cv::Point2d corners1[] ,const std::vector<cv::Point2d>& corners2)
{
    cv::Point2d center1 = ( corners1[0] + corners1[2])/2;
    cv::Point2d distance1_vector = corners1[0] - center1;
    double distance1 = cv::sqrt( distance1_vector.x*distance1_vector.x +
                                    distance1_vector.y*distance1_vector.y );
    cv::Point2d center2 = ( corners2[0] + corners2[2])/2;
    cv::Point2d distance2_vector = corners2[0] - center2;
    double distance2 = cv::sqrt( distance2_vector.x*distance2_vector.x +
                                         distance2_vector.y*distance2_vector.y );
    return distance1 > distance2;

}

void getCorners(const cv::Mat& src , std::vector<cv::Point2d>& corners_max ,double error_fraction)
{
    const std::string family_str = "Tag16h5";
    TagFamily family(family_str);
    TagDetectorParams params;
    params.adaptiveThresholdRadius += (params.adaptiveThresholdRadius+1) % 2;
    if (error_fraction >= 0 && error_fraction < 1) {
        family.setErrorRecoveryFraction(error_fraction);
    }

    TagDetector detector(family, params);
    detector.debug = false;
//    detector.debugWindowName = win;
    TagDetectionArray detections;
    cv::Point2d opticalCenter(0.5*src.rows, 0.5*src.cols);
    clock_t start = clock();
    detector.process(src, opticalCenter, detections);
    clock_t end = clock();
    
    std::cout <<  "\033[1;32mArpilTag\033[0m:\t\t" << "Got " << detections.size() << " detections in "
    << double(end-start)/CLOCKS_PER_SEC << " seconds.\n";
//        cv::Mat img = family.superimposeDetections(src, detections);
//        labelAndWaitForKey(win, "Detected", img, ScaleNone, true);

    for (size_t i=0; i<detections.size(); ++i) {
        cv::Mat img = family.detectionImage(detections[i], src.size(), src.type());
        cv::Point2d corners_tmp[4];
        clock_t start = clock();
        getInitCorners(img,corners_tmp);
        if( compareCorners(corners_tmp , corners_max) )
        {
            for(int j = 0; j < 4; j++)
            {
                corners_max[j] = corners_tmp[j];
            }

            clock_t end = clock();
            // std::cout << "Got corners in "
            //           << double(end-start)/CLOCKS_PER_SEC << " seconds.\n";
        }
    }
    return;
}
//int main(int argc, char** argv) {

//    const std::string win = "Single tag test";
//    double error_fraction ;
//    std::stringstream ss;
//    ss << argv[1];
//    ss >> error_fraction;

//    char file_name[100] ;
//    for (int i = 0; i < 9; ++i) {
//        cv::Point2d corners[4];
//        sprintf(file_name, "/home/wang/wang/git_files/apriltags-cpp/images/image_orig/%d.jpg", i);
//        cv::Mat src = cv::imread(file_name);
//        if (src.empty()) { continue; }
//        getCorners(src,corners,error_fraction);
//        for (uint32_t i = 0; i < 4; i++)
//        {
//            uint32_t next = (i== 4-1)?0:i+1;
//            cv::Point2d p0 = corners[i];
//            cv::Point2d p1 = corners[next];
//            // draw vertex
//            cv::circle(src, p0, double(src.rows) /150 , cv::Scalar(0,0,255),src.rows/150 + 3);
//            // draw line
//            cv::line(src,p0, p1, cv::Scalar(0,255,0) , src.rows/200);
//        }
//        labelAndWaitForKey(win, "Detected", src, ScaleNone, true);


//    }
//    std::cout<<std::endl;
//    return 0;
//}
