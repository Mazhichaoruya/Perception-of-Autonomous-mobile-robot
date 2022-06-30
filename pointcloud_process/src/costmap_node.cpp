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
#include "fstream"
ros::Publisher pub_costmap;
ros::Subscriber pointcloud_sub;
ros::Subscriber map_sub;
int  costmap_width=800,costmap_height=800;
float costmap_resolution=0.01;
float plan_tolerance = 0.05;
float radius = 0.20;
int occupy_thresh=100;
std::string global_frame("world");
std::string local_frame("base_link");

geometry_msgs::TransformStamped transform;
Eigen::Matrix3d r;
Eigen::Vector3d t;

MapGet::MapConverter map_converter,costmap_converter;
nav_msgs::OccupancyGrid  costmap_msg;
matrix global_map ;

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
        float dis=point_.x*point_.x+point_.y*point_.y;
        if(point_.z<1.5&&point_.z>-0.1&&(point_.x>0|| dis>0.2))
            cloud.points.push_back(p);
    }
}
void CostmapCallBack(const sensor_msgs::PointCloud2ConstPtr & msg){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d Tran;
    matrix cost_map;
    float ox = t.x();//机器人坐标原点
    float oy = t.y();
    costmap_msg.info.height=costmap_height;
    costmap_msg.info.width=costmap_width;
    costmap_msg.info.resolution=costmap_resolution;
    costmap_msg.info.origin.orientation=transform.transform.rotation;
    costmap_msg.info.origin.position.x=transform.transform.translation.x;    costmap_msg.info.origin.position.y=transform.transform.translation.y;    costmap_msg.info.origin.position.z=transform.transform.translation.z;
    costmap_msg.data.resize(costmap_msg.info.height*costmap_msg.info.width,0);
//    pcl::fromROSMsg(msg,cloud);
    PointCloudConvert(cloud,msg,false);
    auto t1=ros::Time::now();
    for(auto point:cloud.points){
        int col = (point.x-ox)/costmap_msg.info.resolution;
        int row = (point.y-oy)/costmap_msg.info.resolution;
        if(abs(row)<costmap_msg.info.height/2&& abs(col)<costmap_msg.info.width/2){//&&row>=0&&col>=0
            costmap_msg.data[(row+costmap_msg.info.height/2)*costmap_msg.info.width+(col+costmap_msg.info.width/2)]=255;
        }
    }
    costmap_msg.info.origin.position.x=ox-costmap_msg.info.height/2*costmap_msg.info.resolution;
    costmap_msg.info.origin.position.y=oy-costmap_msg.info.width/2*costmap_msg.info.resolution;
    costmap_msg.info.origin.position.z=0;
    costmap_msg.info.origin.orientation.x=0;costmap_msg.info.origin.orientation.y=0;costmap_msg.info.origin.orientation.z=0;costmap_msg.info.origin.orientation.w=1;
    costmap_converter.SetMapParam(costmap_msg, cost_map, plan_tolerance, radius, occupy_thresh, false);
    for(int i=0;i<costmap_msg.info.width; i++)
        for(int j=0;j<costmap_msg.info.height;j++)
        {
            costmap_msg.data[i+j*costmap_msg.info.width]=cost_map.GetElem(i,j);
        }
    auto t2=ros::Time::now();
    costmap_msg.header.frame_id="/world";
    costmap_msg.header.stamp=msg->header.stamp;
    pub_costmap.publish(costmap_msg);
    costmap_msg.data.clear();
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

    pointcloud_sub = nh.subscribe("/livox_full_cloud_mapped", 1, CostmapCallBack);

    // Advertise topics
    pub_costmap=nh.advertise<nav_msgs::OccupancyGrid>("/costmap",10);
    ros::Rate loop_rate(10);
    ros::param::get("costmap_node/car_radius", radius);
    ros::param::get("costmap_node/plan_tolerance",   plan_tolerance);
    ros::param::get("costmap_node/occupy_thresh", occupy_thresh);
    ros::param::get("costmap_node/costmap_width",costmap_width);
    ros::param::get("costmap_node/costmap_height", costmap_height);
    ros::param::get("costmap_node/costmap_resolution", costmap_resolution);

    ROS_INFO("car_radius: %f",radius);
    ROS_INFO("plan_tolerance :%f",plan_tolerance);
    ROS_INFO("costmap_resolution:%f m",costmap_resolution);
    ROS_INFO("costmap_height:%f m",costmap_height*costmap_resolution);
    ROS_INFO("costmap_width:%f m",costmap_width*costmap_resolution);
    while(ros::ok())
    {
        try
        {
            transform = tfBuffer.lookupTransform(global_frame,local_frame, ros::Time(0));
            GetRotationMatrix(transform);
            Eigen::Quaterniond quat;
            quat.x()=transform.transform.rotation.x,quat.y()=transform.transform.rotation.y,quat.z()=transform.transform.rotation.z,quat.w()=transform.transform.rotation.w;
            Eigen::Vector3d euler=quat.matrix().eulerAngles(2,1,0);

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

