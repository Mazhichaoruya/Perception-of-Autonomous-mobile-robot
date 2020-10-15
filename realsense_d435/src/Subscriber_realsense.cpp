//
// Created by mzc on 2020/9/25.
//
//#include "include.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "realsense_d435/objection.h"
#include "realsense_d435/objectionsofonemat.h"
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
using namespace std;
class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to subscribe
        sub_object = n_.subscribe("Objects", 10, &SubscribeAndPublish::callback, this);
//        sub_odom=n_.subscribe("/odom",5,&SubscribeAndPublish::callback_cloud, this);
        //Topic you want to publish
        pub_pointcloud = n_.advertise<sensor_msgs::PointCloud2>("Point_cloud", 10);
    }

    void callback(const realsense_d435::objectionsofonemat &Objections)
    {
        ROS_INFO("The Objection has:%d ",Objections.sizeofobjections,":\n" );
        for(auto objection:Objections.objectionsofonemat){
            ROS_INFO(objection.classname.data(),":");
            cout<<"CenterPoint:"<<objection.center_point.x<<" "<<objection.center_point.y<<" "<<objection.center_point.z<<endl;
        }
        pointcloud=Objections.pointcloud;
        pointcloud.header.frame_id="realsense";
        pub_pointcloud.publish(pointcloud);
    }
//    void callback_cloud(const nav_msgs::Odometry odometry){
//
//
//    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_pointcloud;
    ros::Subscriber sub_object,sub_odom;
    sensor_msgs::PointCloud2 pointcloud;

};

int main(int argc, char** argv) {
    ros::init(argc,argv,"Object_views");
    SubscribeAndPublish SuPuObjection;
    ros::spin();
    return  0;
}