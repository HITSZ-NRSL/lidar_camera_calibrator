/**
  ******************************************************************************
  * @file	data_reader.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-20
  * @brief	data_reader.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "data_reader.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QStringList>


#include <string>

#include <pcl/io/pcd_io.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

const QString DataReader::kPointcloudFolderName ("pointcloud");
const QString DataReader::kImageFolderName("image_orig");
//const QString DataReader::kLaserscanFolderName("laser");
//const QString DataReader::kTransformFileNmae ("tf.txt");


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

DataReader::DataReader(const QString& path):
    dataset_root_(path),
    image_extension_("jpeg"),
    is_valid_(false),
    flag_kd_(0x00),
    index_(0),
    size_(0)
{
    size_ = getDataNum(path);
    is_valid_ = size_>0 ? true:false;

    K_ = cv::Mat(3, 3, CV_32FC1);
    D_ = cv::Mat(5,1,CV_32FC1);
    tag_size_.resize(3);
}


uint32_t DataReader::getDataNum(const QString &path)
{
	QDir dir(path);

	if(!dir.exists())
	{
		return false;
	}

    QStringList items = dir.entryList(QDir::Dirs|QDir::Files|QDir::NoDotAndDotDot);

    if(!items.contains(kPointcloudFolderName))
    {
        return 0;
    }
    if(!items.contains(kImageFolderName))
    {
        return 0;
    }

//    if(!items.contains(kLaserscanFolderName))
//    {
//        return 0;
//    }
//    if(!items.contains(kTransformFileNmae))
//    {
//        return 0;
//    }

    QStringList imgs = QDir(path + QDir::separator() + kImageFolderName)
                       .entryList(QDir::Files|QDir::NoDotAndDotDot);
    if(imgs.size() > 0)
    {
        QFileInfo fi(imgs[0]);
        image_extension_ = fi.completeSuffix();
    }

    return imgs.count();
}



pcl::PointCloud<pcl::PointXYZI>::Ptr DataReader::getPointcloud()
{
    QString fp = dataset_root_ + QDir::separator() + kPointcloudFolderName +
                 QDir::separator() + QString("%1.pcd").arg(index_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile(fp.toStdString(), *pc) == 1)
    {
        pc = nullptr;
    }
    return pc;
}

std::shared_ptr<cv::Mat> DataReader::getImage()
{
    if(flag_kd_ &0x03 != 0x03)
    {
        // K, D not set
        return nullptr;
    }

    QString fp = dataset_root_ + QDir::separator() + kImageFolderName +
                 QDir::separator() + QString("%1.").arg(index_) + image_extension_;

    cv::Mat img_orig = cv::imread(fp.toStdString(), cv::IMREAD_COLOR);
    if(img_orig.data == NULL)
    {
        std::cerr << "Fail to read " << fp.toStdString() << std::endl;
        return nullptr;
    }
    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>();
    cv::undistort(img_orig, *img, K_, D_);
    return img;
}

bool DataReader::moveNext()
{
    if(index_ >= (size_-1))
    {
        return false;
    }
    index_++;
    return true;
}

void DataReader::setCameraK(const Eigen::Matrix3d &k)
{
    K_ = cv::Mat(3,3,CV_32FC1);
    for(uint8_t i=0; i<9; i++)
    {
        K_.ptr<float>(i/3)[i%3] = k(i/3,i%3);
    }
    flag_kd_ |= 0x01;
}

void DataReader::setCameraD(const Eigen::Matrix<double,5,1>& d)
{
    D_ = cv::Mat(5,1,CV_32FC1);

    D_.ptr<float>(0)[0] = d(0);
    D_.ptr<float>(1)[0] = d(1);
    D_.ptr<float>(2)[0] = d(2);
    D_.ptr<float>(3)[0] = d(3);
    D_.ptr<float>(4)[0] = d(4);

    flag_kd_ |= 0x02;
}

void DataReader::setTagSize(const Eigen::Matrix<double,3,1>& t){
    tag_size_[0] = t(0);
    tag_size_[1] = t(1);
    tag_size_[2] = t(2);
    flag_kd_ |= 0x04;   // tag size设置的标志位
}


/*****************************END OF FILE**************************************/
