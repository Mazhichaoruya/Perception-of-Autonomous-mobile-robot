#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include "nav_msgs/OccupancyGrid.h"
#include "map/map.h"
#include "cluster/cluster.hpp"
#include "fstream"
ros::Publisher pointcloud_pub;
ros::Subscriber pointcloud_sub;
ros::Subscriber map_sub;

std::string global_frame("world");
std::string local_frame("base_link");

std::ofstream odom_fs("/home/mzc/codes/ROS/perception_ws/files/odom/odom.txt");
geometry_msgs::TransformStamped transform;
Eigen::Matrix3d r;
Eigen::Vector3d t;

MapGet::MapConverter map_converter;
matrix global_map ;
bool map_init= false;

void GetRotationMatrix(geometry_msgs::TransformStamped transform)
{
    t[0] = transform.transform.translation.x;
    t[1] = transform.transform.translation.y;
    t[2] = transform.transform.translation.z;
    Eigen::Quaterniond q;  
    q.x() = transform.transform.rotation.x;  
    q.y() = transform.transform.rotation.y;  
    q.z() = transform.transform.rotation.z;  
    q.w() = transform.transform.rotation.w;
    r = q.normalized().toRotationMatrix();
}

std::vector<pcl::PointCloud<pcl::PointXYZI>> clusterCustom(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float tolerance,int min_pts,int max_pts){
    std::vector<pcl::PointCloud<pcl::PointXYZI>> result;
    if(cloud->points.size()<1)
        return result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);//创建点云索引向量，用于存储实际的点云信息
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (tolerance); //设置近邻搜索的搜索半径为5cm
    ec.setMinClusterSize (min_pts);//设置一个聚类需要的最少点数目为3
    ec.setMaxClusterSize (max_pts); //设置一个聚类需要的最大点数目为180
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            auto p=cloud->points[*pit];
            pcl::PointXYZI point;
            point.x=p.x;point.y=p.y;point.z=p.z;point.intensity=j;
            cloud_cluster.points.push_back(point);
        }
        j++;
        cloud_cluster.width = cloud_cluster.points.size();
        cloud_cluster.height = 1;
        cloud_cluster.is_dense = true;
        result.push_back(cloud_cluster);
    }
    return result;
}
//void PointCloudConvert(pcl::PointCloud<pcl::PointXYZI> & cloud,const sensor_msgs::PointCloud2ConstPtr& msg,bool to_world){
//    pcl::PointCloud<pcl::PointXYZI>  cloud_origin;
//    pcl::fromROSMsg(*msg, cloud_origin);
//    for (auto p:cloud_origin) {
//        //指定滤除的机器人高度
//        if(p.z>1.5||p.z<-0.1)
//            continue;
//        Eigen::Vector3d point_vec{p.x,p.y,p.z},point_converted{p.x,p.y,p.z};
//        if (to_world){
//            point_converted=r*point_vec+t;//转换坐标系到世界坐标系
//        }
//        pcl::PointXYZI point_;
//        point_.x=point_converted.x();
//        point_.y=point_converted.y();
//        point_.z=point_converted.z();
//        point_.intensity=p.intensity;
//        cloud.points.push_back(point_);
//    }
//}
void PointCloudConvert(pcl::PointCloud<pcl::PointXYZI> & cloud,const sensor_msgs::PointCloud2ConstPtr & msg,bool to_body){
    pcl::PointCloud<pcl::PointXYZI>  cloud_origin;
    pcl::fromROSMsg(*msg, cloud_origin);
    for (auto &p:cloud_origin) {
        //指定滤除的机器人高度

        Eigen::Vector3d point_vec{p.x,p.y,p.z},point_converted{p.x,p.y,p.z};
        if (to_body){
            point_converted=r.inverse()*(point_vec-t);//转换坐标系到世界坐标系
        }
        pcl::PointXYZI point_;
        point_.x=point_converted.x();
        point_.y=point_converted.y();
        point_.z=point_converted.z();
        point_.intensity=p.intensity;
        if(point_.z<1.5&&point_.z>-0.1)
            cloud.points.push_back(p);
    }
}
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    //载入全局地图
    if(map_init)
        return;;
    map_converter.SetMapParam(msg, global_map, 0, 0, 100, false);
    map_init=true;
//    std::cout<<"map init!"<<std::endl;
}
void DataInCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!map_init)
        return;
    sensor_msgs::PointCloud2 msg_clustered;

    pcl::PointCloud<pcl::PointXYZI> full_cloud;
    //原始点云提取后转换到世界坐标系
    // 对比静态地图，动态新加入的点
//    std::cout<<"data init!"<<std::endl;
//    pcl::fromROSMsg(*msg, full_cloud);

    PointCloudConvert(full_cloud,msg, false);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_out_cloud(new pcl::PointCloud<pcl::PointXYZI> );

    for (auto &point:full_cloud) {
        int index_x,index_y;
        map_converter.WorldToMap(index_x,index_y,point.x,point.y);
        if(map_converter.CheckReachable(index_x,index_y,global_map,50)){
            map_out_cloud->points.push_back(point);
        }
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (map_out_cloud);//创建点云索引向量，用于存储实际的点云信息
    std::vector<pcl::PointIndices> cluster_indices;
    Cluster<pcl::PointXYZI> cluster;
    cluster.setClusterParam(0.2,2,10000,tree);
    cluster.initSupervisedInfo(0);//设置监督信息 从历史信息中获取
    cluster.setInputCloud (map_out_cloud);
    cluster.extractCloud (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
    int j=0;
    pcl::PointCloud<pcl::PointXYZI> cloud_clustered;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            auto p=map_out_cloud->points[*pit];
            pcl::PointXYZI point;
            point.x=p.x;point.y=p.y;point.z=p.z;point.intensity=j;
            cloud_clustered.points.push_back(point);
        }
        j++;
    }
    pcl::toROSMsg(cloud_clustered,msg_clustered);
    msg_clustered.header.stamp=msg->header.stamp;
    msg_clustered.header.frame_id=global_frame;
    pointcloud_pub.publish(msg_clustered);

}

int main(int argc, char * argv[])
{

    //  Initial node
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::vector<double> vecTlb;
	ros::param::get("~Extrinsic_Tlb",vecTlb);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener(tfBuffer);
    // Subscribe topics

    pointcloud_sub = nh.subscribe("/livox_full_cloud_mapped", 1, DataInCallback);
    map_sub = nh.subscribe("/map", 10, MapCallback);

    // Advertise topics
    pointcloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/clustered_points",10);
//    ROS_INFO("Start Obstacle Avoidance!\n");
    // Loop and wait for callback
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        try
        {
            transform = tfBuffer.lookupTransform(global_frame,local_frame, ros::Time(0));
            GetRotationMatrix(transform);
            Eigen::Quaterniond quat;
            quat.x()=transform.transform.rotation.x,quat.y()=transform.transform.rotation.y,quat.z()=transform.transform.rotation.z,quat.w()=transform.transform.rotation.w;
            Eigen::Vector3d euler=quat.matrix().eulerAngles(2,1,0);
//             std::cout<<transform.transform.translation.x<<","<<transform.transform.translation.y<<","<<transform.transform.translation.z<<",,,,,,,"<<std::endl;
            odom_fs<<transform.transform.translation.x<<" "<<transform.transform.translation.y<<" "<<transform.transform.translation.z<<" "<<euler(0)<<" "<<euler(1)<<" "<<euler(2)<<std::endl;

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

