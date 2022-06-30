#include <ros/ros.h>
#include "iostream"
#include "vector"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
std::vector<cv::VideoCapture> caps;
std::vector<ros::Publisher> publishers;
bool cv_show= false;
void timerCallBack(const ros::TimerEvent &e)
{
    std::vector<cv::Mat> images;
    int id=0;
    for (auto cap:caps) {
        cv::Mat image;
        if(cap.read(image)){
            sensor_msgs::ImagePtr img_msg=cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8,image).toImageMsg();
            img_msg->header.frame_id="camera"+std::to_string(id);
            img_msg->header.stamp=ros::Time::now();
            publishers[id].publish(img_msg);
            if (cv_show)
                cv::imshow("image"+std::to_string(id),image);
        }
        id++;
    }
    for (auto &img:images){


    }
}

int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    int num_cam=1;
    int width=1280;
    int height=720;
    int FPS=10;
    ros::param::get("cams_recorder_node/cam_num",num_cam);
    ros::param::get("cams_recorder_node/image_width",width);
    ros::param::get("cams_recorder_node/image_height",height);
    ros::param::get("cams_recorder_node/image_fps",FPS);
    ros::param::get("cams_recorder_node/cv_show",cv_show);
    caps.resize(num_cam);
    publishers.resize(num_cam);
    for (int i=0;i<caps.size();i++ ){
        caps[i].open(i*2);
        caps[i].set(cv::CAP_PROP_FPS,FPS);
        caps[i].set(cv::CAP_PROP_FOURCC,cv::CAP_IMAGES);
        caps[i].set(cv::CAP_PROP_FRAME_WIDTH,width);
        caps[i].set(cv::CAP_PROP_FRAME_HEIGHT,height);
        publishers[i]= nh.advertise<sensor_msgs::Image>("/image_raw"+std::to_string(i), 1);;
    }

    ros::Timer timer=nh.createTimer(ros::Duration(0.1),timerCallBack);

    // Loop and wait for callback
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        loop_rate.sleep();
        cv::waitKey(1);
        ros::spinOnce();
    }
    return 0;
}

