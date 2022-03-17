/**
 * @file SolvePnP.hpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#ifndef SOLVEPNP_HPP
#define SOLVEPNP_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iostream>

using namespace std;
using namespace cv;

/**
 * @brief use cv::solvePnP to get initial calibration parameters, then use g2o to optimize PnP problem
 * 
 * @param _points_2d 2d points
 * @param _points_3d 3d points
 * @param fx intrinsic parameters
 * @param fy 
 * @param cx 
 * @param cy 
 * @param debug_flag output debug log 
 * @param verbose_flag output verbose debug log
 * @return Eigen::Matrix4d transformation from lidar to camera
 */
Eigen::Matrix4d solvePnPbyG2O(const Eigen::Matrix3Xd& _points_2d, const Eigen::Matrix4Xd& _points_3d,
                    double fx, double fy, double cx,double cy, bool debug_flag = false, bool verbose_flag = false){
    assert(_points_2d.cols() == _points_3d.cols());
    int data_num = _points_2d.cols();
    if(debug_flag){
        cout << "fx: " << fx << endl;
        cout << "fy: " << fy << endl;
        cout << "cx: " << cx << endl;
        cout << "cy: " << cy << endl;
        cout << _points_2d.matrix() << endl;
        cout << _points_3d.matrix() << endl;
    }
    vector<Point3f> pts_3d, pts_3d_initial;
    vector<Point2f> pts_2d, pts_2d_initial;
    int initial_data_num = min(data_num, 4);
    for(int i = 0; i < initial_data_num; ++i){
        pts_3d_initial.emplace_back(_points_3d(0,i), _points_3d(1,i), _points_3d(2,i));
        pts_2d_initial.emplace_back(_points_2d(0,i), _points_2d(1,i));
    }
    for(int i = 0; i < data_num; ++i){
        pts_3d.emplace_back(_points_3d(0,i), _points_3d(1,i), _points_3d(2,i));
        pts_2d.emplace_back(_points_2d(0,i), _points_2d(1,i));
    }


    Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    Mat r(3,1,CV_32F), t(3,1,CV_32F);
    solvePnP ( pts_3d_initial, pts_2d_initial, K, Mat(), r, t, false ); 
    Mat R;
    cv::Rodrigues ( r, R ); 

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolver_6_3;
    std::unique_ptr<BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<BlockSolver_6_3::PoseMatrixType>>(); 
    std::unique_ptr<BlockSolver_6_3> solver_ptr ( new BlockSolver_6_3(std::move(linearSolver)) );
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(verbose_flag); 

    // vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); 
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId(0);
    pose->setFixed(false);
    // pose->setEstimate(g2o::SE3Quat(
    //     R_mat, Eigen::Vector3d(-0.1638, -0.0442, 0.0155)));
    pose->setEstimate ( g2o::SE3Quat (
                        R_mat,
                        Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                    ) );
    optimizer.addVertex(pose);

    // edges
    // const float deltaMono = sqrt(5.991);
    for (int i = 0; i < data_num; ++i)
    {
        g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        Eigen::Matrix<double,2,1> obs;
        obs << pts_2d[i].x, pts_2d[i].y ;
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());

        // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        // e->setRobustKernel(rk);
        // rk->setDelta(deltaMono);

        e->fx = fx;
        e->fy = fy;
        e->cx = cx;
        e->cy = cy;
        e->Xw[0] = pts_3d[i].x;
        e->Xw[1] = pts_3d[i].y;
        e->Xw[2] = pts_3d[i].z;

        optimizer.addEdge(e);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    g2o::SE3Quat pose_estimate = pose->estimate();

    if(debug_flag){
        cout<<"initial R="<<endl<<R<<endl;
        cout<<"initial t="<<endl<<t<<endl;
        cout << "SolvePnP:\t" << "solve time cost = " << time_used.count() << " seconds. " << endl;
        cout << "SolvePnP:\t" << "estimated model: \n" << pose_estimate.to_homogeneous_matrix().matrix() << endl;
    }

    return pose_estimate.to_homogeneous_matrix();
}


#endif // SOLVEPNP_HPP