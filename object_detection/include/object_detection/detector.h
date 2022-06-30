//
// Created by mzc on 2021/1/30.
//

#ifndef CLOUD_SAVE_DETECTOR_H
#define CLOUD_SAVE_DETECTOR_H

#include "cuda_runtime_api.h"
#include "logging.h"
#include "opencv2/opencv.hpp"
#define USE_FP16  // comment out this if want to use FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define NET s  // s m l x
#define NETSTRUCT(str) createEngine_##str
#define CREATENET(net) NETSTRUCT(net)
#define STR1(x) #x
#define STR2(x) STR1(x)
struct Object{
    std::string name;
    int classID;
    cv::Rect area;
};
class Detector {
private:
    int inputIndex;
    int outputIndex;
    void* buffers[2];
    cudaStream_t stream;
    std::vector<std::string> classNamesVec;//类别名池
    std::vector<Object> Objects;
    int objectnums;
public:
    Detector();
    Detector(std::string weight_path,std::string classname_path);
    cv::Mat Detection(cv::Mat &img);
    std::vector<Object> ReportObjects();
    void destory();
    ~Detector();
};


#endif //CLOUD_SAVE_DETECTOR_H
