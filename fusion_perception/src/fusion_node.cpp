//
// Created by mzc on 2022/3/22.
//

#include <ros/ros.h>
#include "tracker/tracker2d.hpp"
#include "tracker/tracker3d.hpp"
#include "iostream"
#include "vector"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "fusion_perception/Frame.h"
#include "fusion_perception/Image.h"
#include "fusion_perception/Track.h"
#include "fusion/fusion.hpp"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "visualization/visualization.hpp"
Fusion fusion;
Projection projection;
cv::Size size_image{640,480};
Tracker2D tracker2D(2,2,0.6,0.1);
Tracker3D tracker3D(2,3,0.8,0.1);
bool resume= false;
Eigen::Matrix4f extraTwb= Eigen::Matrix4f::Identity() ;
std::vector<Track2d> imageTs;
std::vector<Track3d> pointsTs;
ros::Publisher pub_frame,pub_image,pub_points;
ros::Publisher classname_pub,arrow_pub,center_pub;
std_msgs::Header publish_header;
std::array<float ,6> calculateBbox(pcl::PointCloud<pcl::PointXYZI> &clouds){
    int size=clouds.points.size();
    float max_x=INT32_MIN,max_y=INT32_MIN,max_z=INT32_MIN,min_x=INT32_MAX,min_y=INT32_MAX,min_z=INT32_MAX;
    float sum_x=0,sum_y=0,sum_z=0;
    for (auto p:clouds){
        if (max_x<p.x) max_x=p.x;
        if (max_y<p.y) max_y=p.y;
        if (max_z<p.z) max_z=p.z;
        if (min_x>p.x) min_x=p.x;
        if (min_y>p.y) min_y=p.y;
        if (min_z>p.z) min_z=p.z;
        sum_x+=p.x;
        sum_y+=p.y;
        sum_z+=p.z;
    }
    return std::array<float,6>{sum_x/size,sum_y/size,sum_z/size,max_x-min_x,max_y-min_y,max_z-min_z};
}

void timeCallBack(const ros::TimerEvent &e){
//    std::cout<<"timer callback"<<std::endl;
    // 定时更新用于Track中kf的predict
    if(tracker2D.reportInit())
        tracker2D.predict();
    if(tracker3D.reportInit())
        tracker3D.predict();
    if(tracker2D.reportInit()&&tracker3D.reportInit())
        resume=true;
}

void clustersCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
//    std::cout<<"points callback"<<std::endl;
    //接受聚类后的点云 对3D tracker 进行状态更新
    publish_header=msg->header;
    pcl::PointCloud<pcl::PointXYZI> cluster_cloud;
    pcl::fromROSMsg(*msg,cluster_cloud);
    std::vector<Cluster> clusters;
    std::map<int,pcl::PointCloud<pcl::PointXYZI>> cluster_map;
    for (auto point:cluster_cloud.points) {
        cluster_map[point.intensity].points.push_back(point);
    }
    for (auto clu:cluster_map) {
        Cluster cluster_cur;
        cluster_cur.classID=0;
        cluster_cur.pointCloud=clu.second;
        cluster_cur.bbox3d= calculateBbox(cluster_cur.pointCloud);
        clusters.push_back(cluster_cur);
    }
//    std::cout<<"cur_size:"<<clusters.size()<<std::endl;
    if (!tracker3D.reportInit()){
        double_t  time = msg->header.stamp.toSec();
        tracker3D.trackInit(clusters,time);
        return;
    }else{
        tracker3D.update(clusters);
    }
}

void debugCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
//    std::cout<<"points callback"<<std::endl;
    //接受聚类后的点云 对3D tracker 进行状态更新
    pcl::PointCloud<pcl::PointXYZI> cluster_cloud;
    pcl::fromROSMsg(*msg,cluster_cloud);
    cv::Mat rebuild=cv::Mat::zeros(size_image.height,size_image.width,CV_8UC(1));
    cv::Mat rebuild_back=cv::Mat::zeros(size_image.height,size_image.width,CV_8UC(1));

    for (auto p:cluster_cloud.points) {
        Eigen::Vector3f point;
        point<<p.x,p.y,p.z;
        auto pix=projection.pointConvertToPix(point);
        if(pix.x()>0&&pix.x()<640&&pix.y()>0&&pix.y()<480/*&&pix.z()>0*/){
//            std::cout<<pix.z();
            int value=(10-abs(pix.z()))*25;
            if(pix.z()>0)
                rebuild.at<uint8_t>(pix.y(),pix.x())=value;
            else
                rebuild_back.at<uint8_t>(pix.y(),pix.x())=value;
        }
    }
    cv::imshow("rebuild",rebuild);
//    cv::imshow("rebuild_back",rebuild_back);

}

void objectCallBack(const object_detection::ObjectsConstPtr &msg){
    //接受2D目标检测结果 对D tracker 进行状态更新
//    std::cout<<"objects callback"<<std::endl;
    object_detection::Objects  objects;
    objects = *msg;
    if (!tracker2D.reportInit()){
        double_t  time = msg->header.stamp.toSec();
        tracker2D.trackInit(objects,time);
        return;
    }
    else{
        tracker2D.update(objects);
    }
}

void imageCallBack(const sensor_msgs::ImageConstPtr &img_msg)
{
    //可视化跟踪结果
    cv::Mat img=cv_bridge::toCvShare(img_msg,sensor_msgs::image_encodings::BGR8)->image;
    for (size_t j = 0; j < imageTs.size(); j++) {
        auto trc=imageTs[j];
        cv::Rect2d rect2D;
        rect2D.x=trc.state.at<float>(0);
        rect2D.y=trc.state.at<float>(1);
        rect2D.width=trc.state.at<float>(2);
        rect2D.height=trc.state.at<float>(3);

        cv::rectangle(img, rect2D, cv::Scalar(0x27, 0xC1, 0x36), 2);//绘制矩形框
        cv::putText(img,std::to_string(trc.ID)+":"+std::to_string(trc.age), cv::Point(rect2D.x, rect2D.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);//图像上添加ID
    }
    sensor_msgs::ImagePtr image_msg=cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8,img).toImageMsg();
    image_msg ->header = img_msg->header;
    pub_image.publish(image_msg);
}

void odomCallback(const nav_msgs::Odometry& msg)
{
    Eigen::Quaternionf quat;
    quat.x() = msg.pose.pose.orientation.x;
    quat.y() = msg.pose.pose.orientation.y;
    quat.z() = msg.pose.pose.orientation.z;
    quat.w() = msg.pose.pose.orientation.w;
    quat = quat.normalized().toRotationMatrix();
    Eigen::Matrix3f rot=quat.normalized().toRotationMatrix();
    Eigen::Vector3f trans;
    trans<<msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z;
    extraTwb.block<3,3>(0,0)=rot;
    extraTwb.block<3,1>(0,3)=trans;
    projection.setExtraTwb(extraTwb);
}
int main(int argc, char * argv[])
{

    //  Initial node
    ros::init(argc, argv, "track");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber sub_objects,sub_cluster,sub_odom,sub_debug,sub_img,sub_points;

    float dt=0.1;
    ros::param::get("fusion_node/dt",dt);

    ros::Timer timer = nh.createTimer(ros::Duration(dt), timeCallBack);

    sub_cluster=nh.subscribe<sensor_msgs::PointCloud2>("/clustered_points",1,clustersCallback);
    sub_debug=nh.subscribe<sensor_msgs::PointCloud2>("/clustered_points",1,debugCallback);
    sub_img = nh.subscribe<sensor_msgs::Image>("/image_raw0",10,imageCallBack);
    sub_objects=nh.subscribe<object_detection::Objects>("/results_yolo",1,objectCallBack);
    sub_odom = nh.subscribe("/livox_odometry_mapped",1,odomCallback);
    pub_image=nh.advertise<sensor_msgs::Image>("/track_images",1);
    center_pub = nh.advertise<visualization_msgs::Marker>("/object_center", 10);
    classname_pub=nh.advertise<visualization_msgs::MarkerArray>("/object_classnames", 10);
    arrow_pub = nh.advertise<visualization_msgs::MarkerArray>("/object_vel", 10);
    // Loop and wait for callback
    ros::Rate loop_rate(10);
    projection.setModel(0);
    projection.setImageSize(size_image);
//    fusion.fusionAgnesInit(size_image,0,0.2);
    Visualization3D visualization3D;
    visualization3D.initMarker();
    while(ros::ok())
    {
        if (resume){
            //预测更新完成
            resume= false;
            imageTs=tracker2D.reportResults();
            pointsTs=tracker3D.reportResults();
            fusion.setInput(pointsTs,imageTs);
            fusion.fusionHungarian(size_image,0,0.9,extraTwb);
//            auto res=fusion.fusionWithANGES();
            auto frame=fusion.reportResult();

            //Visualization
//            visualization3D.setVisualization(pointsTs,publish_header);

            visualization3D.setVisualization(frame,publish_header);
            visualization_msgs::MarkerArray  arrows=visualization3D.reportArrows();
            visualization_msgs::MarkerArray  classnames=visualization3D.reportClassnames();
            visualization_msgs::Marker  centers=visualization3D.reportCenters();
//            std::cout<<"size:"<<arrows.markers.size()<<" "<<classnames.markers.size()<<std::endl;
            arrow_pub.publish(arrows);
            center_pub.publish(centers);
            classname_pub.publish(classnames);

        }
        loop_rate.sleep();
        ros::spinOnce();
        cv::waitKey(1);
    }
    return 0;
}

