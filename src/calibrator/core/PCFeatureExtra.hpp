/**
 * @file PCFeatureExtra.hpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#ifndef PCFEATUREEXTRA_HPP
#define PCFEATUREEXTRA_HPP


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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iostream>

using namespace std;
using namespace cv;

class PcPolygonFitEdge: public g2o::BaseUnaryEdge<1,double,g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PcPolygonFitEdge(const pcl::PointXYZ& p, const std::vector<double>& tag_size): BaseUnaryEdge(), p_(p), tag_size_(tag_size){
        assert(tag_size_.size() == 3);
    }
    // error 
    void computeError(){
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const g2o::SE3Quat estimate = v->estimate();
        g2o::VectorN<3> p_L, p_T;
        p_L << p_.x, p_.y, p_.z;
        p_T = estimate * p_L;
        _error(0,0) = error_function( p_T.x(), p_T.y(), p_T.z() ) - _measurement;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    pcl::PointXYZ p_;
private:
    std::vector<double> tag_size_;
    double error_function(double x, double y, double z){
        return checkCost(x, -tag_size_[2]/2, tag_size_[2]/2) + 
                checkCost(y, -tag_size_[1]/2, tag_size_[1]/2) + 
                checkCost(z, -tag_size_[0]/2, tag_size_[0]/2) ;
    }
    double checkCost(double value, double min_threshold, double max_threshold){
        if( value >= min_threshold && value <= max_threshold) return 0;
        else {
            return min(abs(value - min_threshold), abs(value - max_threshold));
        }
    }
};

Eigen::Matrix4d extractPCFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, const std::vector<double>& tag_size,bool debug_flag = true, bool verbose_flag = false){
    // initialize g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,1> > Block;  
    std::unique_ptr<Block::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<Block::PoseMatrixType>> (); 
    std::unique_ptr<Block> solver_ptr(new Block ( std::move(linearSolver) ));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( verbose_flag );        

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();    // target pose
    Eigen::Matrix3d R_mat( Eigen::Matrix3d::Identity() );
    pose->setId( 0 );
    pose->setEstimate( g2o::SE3Quat (
        R_mat, Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edge
    int size = _cloud->points.size();
    for(int i = 0; i < size; ++i){
        PcPolygonFitEdge* edge = new PcPolygonFitEdge(_cloud->points[i], tag_size);
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->setMeasurement(0);
        edge->setInformation( Eigen::Matrix<double, 1, 1>::Identity());
        optimizer.addEdge( edge );
    }

    // optimize
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    
    // print 
    g2o::SE3Quat pose_estimate = pose->estimate();
    if(debug_flag){
        cout<< "\033[1;32mPointCloudExtract\033[0m:\t" << "solve time cost = "<<time_used.count()<<" seconds. "<<endl;
        // cout<< "\033[1;32mPointCloudExtract\033[0m:\t" << "estimated model: \n"<<pose_estimate.to_homogeneous_matrix().matrix()<<endl;
    }
    return pose_estimate.to_homogeneous_matrix();
}

#endif // PCFEATUREEXTRA_HPP