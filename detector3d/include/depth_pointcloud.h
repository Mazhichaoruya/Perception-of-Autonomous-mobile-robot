//
// Created by mzc on 2020/12/10.
//

#ifndef SRC_DEPTH_POINTCLOUD_H
#define SRC_DEPTH_POINTCLOUD_H

#include "lidar_pointcloud.h"
#include "include.h"
class depth_pointcloud {
private:
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
    DBSCANKdtreeCluster<pcl::PointXYZI> ec;
    Camera_info cam,camrgb;
public:
    int cloudsize;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;//输入点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f;//降采样后点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster;//聚类后点云
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<Cluster> vec_clu;
    template <typename PointCloudPtrType>
    void show_point_cloud(PointCloudPtrType cloud, std::string display_name);
    void pointcloud_cluster();
    void pointcloud_predeal();
    void init(cv::Mat &depthmat,int id);
};


#endif //SRC_DEPTH_POINTCLOUD_H
