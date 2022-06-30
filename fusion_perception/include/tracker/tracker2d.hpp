#pragma onece
#include "tracker.hpp"
#include "fusion_perception/Track.h"
#include "fusion_perception/Image.h"
#include "object_detection/Objects.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "map"


struct Track2d:public Track{
    KalmanModel kf{8,4};
};

class Tracker2D:public Tracker<Track2d>{
private:
    static const int measures_dim=4;
    static const int state_dim=8;
    Track2d newTrack(object_detection::Object object) ;
    std::vector<std::vector<float>> Iou_cost(object_detection::Objects &det_boxes) ;
    float Iou_calculate(std::array<float,4>  t1,std::array<int,4> t2) ;

public:
    Tracker2D(int birth,int death,float th,float dt_) ;
    void trackInit(object_detection::Objects &objects,double_t time_) ;
    void update(object_detection::Objects &objects) ;
    std::vector<fusion_perception::Image> reportTracks();
    bool reportInit();
    std::vector<Track2d> reportResults();
    double_t reportTime();

};

Tracker2D::Tracker2D(int birth,int death,float th,float dt_) {
    setParam(birth,death,th,dt_);
}

Track2d Tracker2D::newTrack(object_detection::Object object) {
    Track2d track;
    cv::Mat state=cv::Mat::zeros(state_dim,1,CV_32F);
    for (int i=0;i<object.bbox.size();i++) {
        state.at<float>(i)=object.bbox.at(i);
    }
    track.kf.kalmanFilterSetup(dt,1e3,1e-3,1e-2);
    track.kf.kalmaninitstate(state);
    track.age=0;
    track.no_match=0;
    track.enable= false;
    track.ID=trackID;
    track.classID = object.classID;
    track.classname=object.classname;
    trackID++;
    return track;
}

float Tracker2D::Iou_calculate(std::array<float,measures_dim>  t1,std::array<int,measures_dim> t2) {
    float distance=0.0;
    cv::Rect rect1{(int)t1[0],(int)t1[1],(int)t1[2],(int)t1[3]};
//    std::cout<<"text--rect1:"<<rect1<<std::endl;
    cv::Rect rect2{t2[0],t2[1],t2[2],t2[3]};
    float Iou=(rect1 & rect2).area()*1.0/(rect1 | rect2).area();
    return Iou;
}

std::vector<std::vector<float>> Tracker2D::Iou_cost(object_detection::Objects &det_boxes) {
    std::vector<std::vector<float>> costs;
    for(auto track:tracks){
        std::vector<float> cost;
        std::array<float ,measures_dim> track_bbox;
        track_bbox={track.second.state.at<float>(0),track.second.state.at<float>(1),track.second.state.at<float>(2),track.second.state.at<float>(3)};
        for(auto object : det_boxes.objects){
            std::array<int ,measures_dim> object_bbox;
            object_bbox={object.bbox.at(0),object.bbox.at(1),object.bbox.at(2),object.bbox.at(3)};
            float ele = 1 - Iou_calculate(track_bbox, object_bbox);
            cost.push_back(ele);
        }
        costs.push_back(cost);
    }
    return costs;
}

void Tracker2D::trackInit(object_detection::Objects &objects,double_t time_){
    time=time_;
    for (auto object:objects.objects) {
        tracks.insert(std::pair<int,Track2d>(trackID, newTrack(object)));
    }
    init= true;
}


void Tracker2D::update(object_detection::Objects &objects) {
    std::vector<std::vector<float>> cost_matrix= Iou_cost(objects);
    std::vector<int> detIds;
    std::vector<int> takIds;
    for (auto track:tracks) {
        takIds.push_back(track.first);
    }
    if(cost_matrix.empty()){
//        std::cout<<"cost matrix is empty!"<<std::endl;
    }else{
//        std::cout<<"cost:"<<std::endl;
//        for (auto m:cost_matrix) {
//            for (auto e:m) {
//                std::cout<<e<<" ";
//            }
//            std::cout<<std::endl;
//        }
        matcher.Solve(cost_matrix,detIds);
    }
//----------------------------- 首轮遍历 对更新信息进行遍历---------------------------------------------------------------------------------
    for (int i = 0; i <objects.objects.size() ; ++i) {
        auto iter=std::find(detIds.begin(),detIds.end(),i);
        if(iter>=detIds.end()){
            //新出现的目标未有trace对应 需要新建
            auto new_track=newTrack(objects.objects[i]);//新建一个trace
            tracks.insert(std::pair<int,Track2d>(new_track.ID, new_track));
        }else{
            // 有对应trace匹配上
            int cur_trackID=takIds.at(iter-detIds.begin());
            cv::Mat measurement=cv::Mat::zeros(measures_dim,1,CV_32F);
            for (int row=0;row<objects.objects[i].bbox.size();row++) {
                measurement.at<float>(row)=objects.objects[i].bbox[row];
            }
            tracks.at(cur_trackID).kf.kalmanUpdate(measurement);
        }
    }
//---------------------------------次轮遍历 对现有trace信息进行遍历-------------------------------------------------------------------------------
    for (int i = 0; i < takIds.size(); ++i) {
        int detIndex=-1;
        if(detIds.size()>0)
            detIndex=detIds[i];
        int cur_trackId=takIds[i];
        if (detIndex<0||cost_matrix[i][detIndex] > threshd)
        {//匹配失败
//            std::cout<<cur_trackId<<" nomatch"<<std::endl;
            tracks[cur_trackId].no_match++;
        }else{
//            std::cout<<cur_trackId<<" matched"<<std::endl;
            tracks[cur_trackId].no_match=0;
        }
        checkState(cur_trackId);
    }
}

std::vector<fusion_perception::Image> Tracker2D::reportTracks() {
    //执行 对应Tracks的预测
//    predict();
    std::vector<fusion_perception::Image> tracks2d;
    for (auto track:tracks) {
        fusion_perception::Image track2d;
        track2d.classID=track.second.classID;
        track2d.age=track.second.age;
        track2d.trackID=track.second.ID;
        std::cout<<std::endl;
        track2d.position={static_cast<int>(track.second.state.at<float>(0)),static_cast<int>(track.second.state.at<float>(1)),static_cast<int>(track.second.state.at<float>(2)),static_cast<int>(track.second.state.at<float>(3))};
        track2d.velocity={static_cast<int>(track.second.state.at<float>(4)),static_cast<int>(track.second.state.at<float>(5)),static_cast<int>(track.second.state.at<float>(6)),static_cast<int>(track.second.state.at<float>(7))};
        tracks2d.push_back(track2d);
    }
    return tracks2d;

}
bool Tracker2D::reportInit() {
    return init;
}
std::vector<Track2d> Tracker2D::reportResults() {
    std::vector<Track2d> result;
    for (auto track:tracks) {
        if (track.second.enable==true){
//            std::cout<<"state:"<<track.second.classID<<" :"<<std::endl;
//            for (int i=0;i<track.second.state.rows;i++) {
//                std::cout<<track.second.state.at<float>(i)<<" ";
//            }
//            std::cout<<std::endl;
            result.push_back(track.second);
        }
    }
    return result;
}
double_t Tracker2D::reportTime(){
    return time;
}