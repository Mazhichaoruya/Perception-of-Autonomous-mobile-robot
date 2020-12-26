//
// Created by mzc on 2020/11/28.
//

#ifndef SRC_LIDAR_POINTCLOUD_H
#define SRC_LIDAR_POINTCLOUD_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/common.h>
#include <iostream>
#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"
#include "include.h"
struct Cluster {
    cv::Rect R;
    int index;
    int cloudsize;
    pcl::PointXYZ center_point;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcould;
};
struct Camera_info{
    Eigen::Vector3f TLtoC;//雷达和相机和坐标系平移变换
    Eigen::Matrix<float,3,3> RLtoC;//雷达和相机的旋转变换
    Eigen::Matrix<float,3,3> Camerainfo;//相机内参数
    int index,imageheigth,imagewidth;//图像大小
    int max_y,min_y;
};
class lidar_pointcloud {
private:
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
    DBSCANKdtreeCluster<pcl::PointXYZI> ec;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
public:
    int cloudsize;
    std::array<Camera_info,2> cam_vec;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIncamera;
    std::vector<pcl::PointIndices> cluster_indices;
    cv::Mat lidar_clus,lidar_depth;
    std::vector<Cluster> vec_clu;
    lidar_pointcloud();
    template <typename PointCloudPtrType>
    void show_point_cloud(PointCloudPtrType cloud, std::string display_name);
    void pointcloud_cluster();
    void init(pcl::PointCloud<pcl::PointXYZI>::Ptr pointclouds);
    void initcamera(Eigen::Vector3f T,Eigen::Matrix<float,3,3> R,Eigen::Matrix<float,3,3> inner,int H,int W,int id);
};


#endif //SRC_LIDAR_POINTCLOUD_H
