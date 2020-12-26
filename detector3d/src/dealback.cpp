//
// Created by mzc on 2020/9/25.
//
//#include "include.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "detector3d/objectsofonemat.h"
#include <sensor_msgs/PointCloud2.h>
using namespace std;
class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to subscribe
        sub_ = n_.subscribe("Objects", 10, &SubscribeAndPublish::callback, this);
        //Topic you want to publish
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("Point_cloud", 10);
    }

    void callback(const detector3d::objectsofonemat &Objects)
    {
        sensor_msgs::PointCloud2 pointcloud;
        ROS_INFO("The Objection has:%d ",Objects.sizeofobjects,":\n" );
        for(auto objection:Objects.objectsofonemat){
            ROS_INFO(objection.classname.data(),":");
            cout<<"CenterPoint:"<<objection.center_point.x<<" "<<objection.center_point.y<<" "<<objection.center_point.z<<endl;
        }
        pointcloud=Objects.pointcloud;
        pub_.publish(pointcloud);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};

int main(int argc, char** argv) {
    ros::init(argc,argv,"Objection_views");
    SubscribeAndPublish SuPuObjection;
    ros::spin();
    return  0;
}