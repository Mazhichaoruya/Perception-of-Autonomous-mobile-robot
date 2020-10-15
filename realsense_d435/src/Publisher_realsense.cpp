#include "../include/include.h"
#include "time.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "realsense_d435/objection.h"
#include "realsense_d435/objectionsofonemat.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h> // 用来发布和订阅图像信息
using namespace std;
//using namespace cv;
using namespace cv::dnn;
using namespace rs2;
//using namespace mlpack;
const size_t inWidth      = 416;
const size_t inHeight     = 416;
const float WHRatio       = inWidth / (float)inHeight;
const float inScaleFactor = 1/255.f;
const float meanVal       = 127.5;
void image_detection_Cfg();
Mat Dectection(Mat color_mat);
Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
Eigen::Vector3f V_T;//平移向量T
Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
cv::Mat Depthmate,Dec_mat,color_mat;
Net net;//DNN net
std::vector<String> outNames;
rs2::stream_profile dprofile;
rs2::stream_profile cprofile;
vector<string> classNamesVec;
const auto window_name= "RGB Image";
rs2::pipeline pipes;///生成Realsense管道，用来封装实际的相机设备
String yolo_tiny_model ="/home/action/ROS-Object-Detection-2Dto3D-RealsenseD435/Yolo_model/enetb0-coco_final.weights";//yolov4.weights enetb0-coco_final.weights
String yolo_tiny_cfg =  "/home/action/ROS-Object-Detection-2Dto3D-RealsenseD435/Yolo_model/enet-coco.cfg";//yolov4.cfg enet-coco.cfg
String classname_path="/home/action/ROS-Object-Detection-2Dto3D-RealsenseD435/Yolo_model/object_detection_classes_yolov3.txt";
vector<Objection> ObjectionOfOneMat;//一幅图中的目标
int main(int argc, char** argv)
{
    ros::init(argc,argv,"realsense_Object");
    ros::NodeHandle n;
    rs2::colorizer color_map;
    Realsense_config();//相机初始化
    Get_referance();//获取相机内参 和外参
//    set_d435();//相机深度图与彩色图配准
    image_detection_Cfg();//二维目标检测Yolov3初始化
//////////////////////////////////////
    auto config =pipes.start();
    auto profile = config.get_stream(RS2_STREAM_COLOR)
            .as<video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR);
    Size cropSize;
    if (profile.width() / (float)profile.height() > WHRatio)
    {
        cropSize = Size(static_cast<int>(profile.height() * WHRatio),
                        profile.height());
    }
    else
    {
        cropSize = Size(profile.width(),
                        static_cast<int>(profile.width() / WHRatio));
    }
    // Declare filters
    rs2::spatial_filter spat_filter;
    rs2::hole_filling_filter Hole_Filling_filter(1);//孔填充滤波器
    rs2::decimation_filter decimationFilter;
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.55f);
    ///////////////
    ros::Publisher Objection_pub =n.advertise<realsense_d435::objectionsofonemat>("Objects",1000);
//    image_transport::ImageTransport it(n);  //  类似ROS句柄
//    image_transport::Publisher Objection_img=it.advertise("Image",10);;
    ros::Rate loop_rate(10);//发布频率10HZ
    ////////////////////////
    while (ros::ok()) {
        auto data = pipes.wait_for_frames();
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();//.apply_filter(color_map)
//        depth_frame=decimationFilter.process(depth_frame);
//        depth_frame=spat_filter.process(depth_frame);
        depth_frame=Hole_Filling_filter.process(depth_frame);
        // Convert RealSense frame to OpenCV matrix:
        color_mat = frame_to_mat(color_frame);
        Mat depth_mat(Size(640,480),
                        CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        Depthmate = depth_mat;
        ObjectionOfOneMat.clear();//清空上一幅图像的目标
        Dec_mat = Dectection(color_mat);
        ////////////////////////////////转换数据格式为ROS发布的信息格式
        ///////////////////////////////
        realsense_d435::objectionsofonemat Objections;
        realsense_d435::objection objection_new;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (auto objection:ObjectionOfOneMat) {
            if (objection.Enable) {
                pcl::PointXYZRGB p;
                objection_new.classname=objection.Classname;
                objection_new.classID=objection.ClassID;
                p.r=255-objection.ClassID*3;
                p.g=objection.ClassID*3;
                p.b=objection.ClassID*2;
                objection_new.center_point.x=objection.Point_Camera.at(0);
                objection_new.center_point.y=objection.Point_Camera.at(1);
                objection_new.center_point.z=objection.Point_Camera.at(2);
                for (auto i:objection.Real_Point) {
                    geometry_msgs::Point32 point;
                    point.x=i[0]/1000.0;
                    point.y=i[1]/1000.0;
                    point.z=i[2]/1000.0;
                    p.x=point.x;
                    p.y=point.y;
                    p.z=point.z;
                    objection_new.point_cloud.push_back(point);
                    cloud.points.push_back(p);
                }
            }
            Objections.objectionsofonemat.push_back(objection_new);
        }
//        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", Dec_mat).toImageMsg();
//        Objection_img.publish(msg);
        pcl::toROSMsg(cloud,Objections.pointcloud);
        Objections.pointcloud.header.stamp=ros::Time::now();
//        Objections.pointcloud.header.frame_id="realsense";
        Objections.sizeofobjections=ObjectionOfOneMat.size();
        Objection_pub.publish(Objections);//发布信息--一幅图像的信息
        if (waitKey(1) != 27)
            imshow(window_name, Dec_mat);
//        Objections.objectionsofonemat.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
Mat Dectection(Mat color_mat) {  //getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0
    // 加载图像 D435加载
//        color_mat = imread("/home/mzc/code/CLionProjects/DNN435/test_image/195.jpg");

    Mat inputBlob = blobFromImage(color_mat, inScaleFactor, Size(inWidth, inHeight), Scalar(), true, false);
    net.setInput(inputBlob);
    // 检测
    std::vector<Mat> outs;
    net.forward(outs, outNames);
    vector<double> layersTimings;
    double freq = getTickFrequency() / 1000;
    double time = net.getPerfProfile(layersTimings) / freq;
    double FPS=1000/time;
    ostringstream ss;
    ss << "FPS: " << FPS ;
    putText(color_mat, ss.str(), Point(0, 10), 0, 0.5, Scalar(255, 0, 0));
    vector<Rect> boxes;
    vector<int> classIds;
    vector<float> confidences;
    for (size_t i = 0; i < outs.size(); ++i) {
        float *data = (float *) outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > 0.5) {
                int centerX = (int) (data[0] * color_mat.cols);
                int centerY = (int) (data[1] * color_mat.rows);
                int width = (int) (data[2] * color_mat.cols);
                int height = (int) (data[3] * color_mat.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float) confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    vector<int> indices;
    NMSBoxes(boxes, confidences, 0.5, 0.2, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        Rect box = boxes[idx];
        auto ClassID=classIds[idx];
        String className = classNamesVec[classIds[idx]];
        Objection NewObjection(box,ClassID);
        ObjectionOfOneMat.push_back(NewObjection);
        putText(color_mat, className.c_str(), box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
        rectangle(color_mat, box, Scalar(0, 0, 255), 2, 8, 0);
    }
    return color_mat;
}
int Realsense_config(){
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    /// Create librealsense context for managing devices
    rs2::context ctx;
    auto devs = ctx.query_devices();                  ///获取设备列表
    int device_num = devs.size();
    std::cout<<"device num: "<<device_num<<std::endl;///设备数量

    ///我只连了一个设备，因此我查看第0个设备的信息
    /// 当无设备连接时此处抛出rs2::error异常
    rs2::device dev = devs[0];
    ///设备编号，每个设备都有一个不同的编号, 如果连接了多个设备，便可根据此编号找到你希望启动的设备
    char serial_number[100] = {0};
    strcpy(serial_number, dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    printf("serial_number: %s\n",serial_number);

    ///设置从设备管道获取的深度图和彩色图的配置对象
    rs2::config cfg;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    ///默认配置任意一个设备，若要配置指定的设备可以根据设备在设备列表里的序列号进行制定:
    ///int indx = 0; ///表示第0个设备
    ///cfg.enable_stream(RS2_STREAM_COLOR,indx, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8, 30);
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    pipes.start(cfg); ///根据给定的配置启动相机管道

    rs2::frameset data;
    data = pipes.wait_for_frames();///等待一帧数据，默认等待5s

    rs2::depth_frame depth       = data.get_depth_frame(); ///获取深度图像数据
    rs2::video_frame color       = data.get_color_frame();  ///获取彩色图像数据
    dprofile =  depth.get_profile();
    cprofile =  color.get_profile();
    return EXIT_SUCCESS;
}
int Get_referance() //try
{
    ///获取彩色相机内参
    rs2::video_stream_profile cvsprofile(cprofile);
    rs2_intrinsics color_intrin =  cvsprofile.get_intrinsics();
    std::cout<<"\ncolor intrinsics: "<<endl;
    InnerTransformation_Color<<color_intrin.fx,0,color_intrin.ppx,0,color_intrin.fy,color_intrin.ppy,0,0,1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout<<InnerTransformation_Color(i,j)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<"distortion model: "<<color_intrin.model<<std::endl;///畸变模型

    ///获取深度相机内参
    rs2::video_stream_profile dvsprofile(dprofile);
    rs2_intrinsics depth_intrin =  dvsprofile.get_intrinsics();
    std::cout<<"\ndepth intrinsics: "<<endl;
    Inner_Transformation_Depth<<depth_intrin.fx,0,depth_intrin.ppx,0,depth_intrin.fy,depth_intrin.ppy,0,0,1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout<<Inner_Transformation_Depth(i,j)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<"distortion model: "<<depth_intrin.model<<std::endl;///畸变模型

    ///获取深度相机相对于彩色相机的外参，即变换矩阵: P_color = R * P_depth + T
    rs2_extrinsics extrin = dprofile.get_extrinsics_to(cprofile);
    std::cout<<"\nextrinsics of depth camera to color camera: \nrotaion: "<<std::endl;
    MTR<<extrin.rotation[0],extrin.rotation[1],extrin.rotation[2],extrin.rotation[3],extrin.rotation[4],extrin.rotation[5],extrin.rotation[6],extrin.rotation[7],extrin.rotation[8];
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            std::cout<<MTR(i,j)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<"translation: ";
    V_T<<extrin.translation[0],extrin.translation[1],extrin.translation[2];
    for(int i=0;i<3;i++)
        std::cout<<V_T(i)<<"\t";
    std::cout<<std::endl;
    pipes.stop();

    return EXIT_SUCCESS;
}
void image_detection_Cfg() {
    net = readNetFromDarknet(yolo_tiny_cfg, yolo_tiny_model);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);// DNN_BACKEND_INFERENCE_ENGINE DNN_BACKEND_OPENCV 未安装IE库 if you have IntelCore CPU you can chose this Para to accelerate youe model--Openvino;
    net.setPreferableTarget(DNN_TARGET_CPU);
    outNames = net.getUnconnectedOutLayersNames();
    for (int i = 0; i < outNames.size(); i++) {
        printf("output layer name : %s\n", outNames[i].c_str());
    }
    ifstream classNamesFile(classname_path);
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }
}
