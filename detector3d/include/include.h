//
// Created by mzc on 2020/8/7.
//

#ifndef DNN_YOLO_INCLUDE_H
#define DNN_YOLO_INCLUDE_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
//#include <opencv2/dnn.hpp>
#include "cv-helpers.hpp"
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <numeric>
//#include "mlpack/core.hpp"
#define Stride 5 //稀疏化步长
#define Distance_Limit (Stride*Stride*100)
#define HeightCam (480)
#define WidthCam (640)
#define USE_FP16  // comment out this if want to use FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define DepthEnable 1
#define LidarEnable 0
#define SLAMEnable 0
//#define VIDEO_TYPE (3) //0 means laptop camera ;1 means images,2 means Videos,3 means Depthcamera,4 means lidar&camera;
#define NORT (1)//Astra or Xtion have no RT of RGB&Depth Camera sys
#define NET l  // s m l x
#define NETSTRUCT(str) createEngine_##str
#define CREATENET(net) NETSTRUCT(net)
#define STR1(x) #x
#define STR2(x) STR1(x)
extern Eigen::Matrix<float,3,3> MTR,MTR1,R_deptoimg,R_deptoimg1;//相机坐标旋转矩阵
extern Eigen::Vector3f V_T,V_T1,T_deptoimg,T_deptoimg1;//平移向量T
extern Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color,Inner_Transformation_Depth1,InnerTransformation_Color1;// 相机内参
extern cv::Mat Depthmate, color_mat;
extern std::vector<std::string> classNamesVec;
//extern int Stride;
#endif //DNN_YOLO_INCLUDE_H
