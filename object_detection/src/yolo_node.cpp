#include <ros/ros.h>
#include "iostream"
#include "vector"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "object_detection/detector.h"
#include "object_detection/Objects.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
std::vector<cv::VideoCapture> caps;
Detector detector;
std::string weight;//模型路径
std::string classname_path;//类别名路径
ros::Publisher publisher_yolo,publisher_img;
std::vector<ros::Subscriber> sub_img;
void imageCallBack(const sensor_msgs::ImageConstPtr &img_msg){
    cv::Mat img=cv_bridge::toCvShare(img_msg,sensor_msgs::image_encodings::BGR8)->image;
    img=detector.Detection(img);
    auto objects=detector.ReportObjects();
    object_detection::Objects frame;
    for (auto object:objects) {
        object_detection::Object obj_msg;
        obj_msg.classname=object.name;
        obj_msg.bbox={object.area.x,object.area.y,object.area.width,object.area.height};
        obj_msg.classID=object.classID;
        frame.objects.push_back(obj_msg);
    }
    frame.cameraID=img_msg->header.frame_id.at(img_msg->header.frame_id.size()-1)-48;
//    std::cout<<frame.cameraID<<std::endl;
    frame.header.frame_id="base_link";
    frame.header.stamp=img_msg->header.stamp;
    publisher_yolo.publish(frame);
    sensor_msgs::ImagePtr image_msg=cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8,img).toImageMsg();
    image_msg ->header = frame.header;
    publisher_img.publish(img_msg);
//    cv::imshow("Detect"+std::to_string(frame.cameraID),img);
}

int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    int num_cam=1;

    ros::param::get("object_detection_node/engine_path",weight);
    ros::param::get("object_detection_node/name_path",classname_path);
    ros::param::get("object_detection_node/cam_num",num_cam);

    sub_img.resize(num_cam);
    for (int i=0;i<sub_img.size();i++ ){
        sub_img[i] = nh.subscribe<sensor_msgs::Image>("/image_raw"+std::to_string(i),10,imageCallBack);
    }
    detector=Detector(weight,classname_path);
    publisher_yolo= nh.advertise<object_detection::Objects>("/results_yolo", 1);;
    publisher_img = nh.advertise<sensor_msgs::Image>("/image_detected",1);
    // Loop and wait for callback
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        loop_rate.sleep();
        cv::waitKey(1);
        ros::spinOnce();
    }
    detector.destory();
    return 0;
}

