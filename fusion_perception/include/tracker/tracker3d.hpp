//
// Created by mzc on 2022/3/31.
//

#pragma once
#include "kalman/kalman.hpp"
#include "fusion_perception/Track.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "map"
#include "tracker/tracker.hpp"

struct Track3d:public Track{
    KalmanModel kf{12,6};
    std::vector<pcl::PointCloud<pcl::PointXYZI>> historyPointClouds;//3D点云信息
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
};

struct Cluster{
   int classID;
   std::array<float ,6> bbox3d;
   pcl::PointCloud< pcl::PointXYZI> pointCloud;
};

class Tracker3D:public Tracker<Track3d>{
private:
    static const int measures_dim=6;
    static const int state_dim=12;
    Track3d newTrack(Cluster object) ;
    std::vector<std::vector<float>> Iou_cost(std::vector<Cluster> &objects) ;
    float Iou_calculate(std::array<float,6> t1,std::array<float,6> t2) ;

public:
    Tracker3D(int birth,int death,float th,float dt_) ;
    void trackInit(std::vector<Cluster> &objects,double_t time_) ;
    void update(std::vector<Cluster> &objects) ;
    std::vector<fusion_perception::Track> reportTracks();
    bool reportInit();
    std::vector<Track3d> reportResults();
    double_t  reportTime();
};

Tracker3D::Tracker3D(int birth, int death, float th,float dt_) {
    setParam(birth,death,th,dt_);
}

Track3d Tracker3D::newTrack(Cluster object) {
    Track3d track;
    cv::Mat state=cv::Mat::zeros(state_dim,1,CV_32F);
    for (int i=0;i<object.bbox3d.size();i++) {
        state.at<float>(i)=object.bbox3d.at(i);
    }
    track.kf.kalmanFilterSetup(dt,1e3,1e-3,1e-2);
    track.kf.kalmaninitstate(state);
    track.age=0;
    track.no_match=0;
    track.enable= false;
    track.ID=trackID;
    track.classID = object.classID;
    track.pointCloud=object.pointCloud;
    trackID++;
    return track;
}

// 3D Iou计算
float Tracker3D::Iou_calculate(std::array<float,measures_dim> t1,std::array<float,measures_dim> t2) {
    auto area1=t1.at(3)*t1.at(4)*t1.at(5);auto area2=t2.at(3)*t2.at(4)*t2.at(5);
    float x1=std::max(t1.at(0)-t1.at(3)/2,t2.at(0)-t2.at(3)/2);
    float y1=std::max(t1.at(1)-t1.at(4)/2,t2.at(1)-t2.at(4)/2);
    float z1=std::max(t1.at(2)-t1.at(5)/2,t2.at(2)-t2.at(5)/2);
    float x2=std::min(t1.at(0)+t1.at(3)/2,t2.at(0)+t2.at(3)/2);
    float y2=std::min(t1.at(1)+t1.at(4)/2,t2.at(1)+t2.at(4)/2);
    float z2=std::min(t1.at(2)+t1.at(5)/2,t2.at(2)+t2.at(5)/2);
    if (x1>=x2||y1>=y2||z1>=z2)
        return 0;
    float inter_area=(x2-x1)*(y2-y1)*(z2-z1);
    return inter_area/(area1+area2-inter_area);
}

//使用Iou作为匈牙利算法的匹配代价
std::vector<std::vector<float>> Tracker3D::Iou_cost(std::vector<Cluster> &objects) {
    std::vector<std::vector<float>> costs;
    for(auto track:tracks){
        std::vector<float> cost;
        std::array<float ,measures_dim> track_bbox;
        track_bbox={track.second.state.at<float>(0),track.second.state.at<float>(1),track.second.state.at<float>(2),
                track.second.state.at<float>(3),track.second.state.at<float>(4),track.second.state.at<float>(5)};
        for(auto object : objects){
            float ele = 1 - Iou_calculate(track_bbox, object.bbox3d);
            cost.push_back(ele);
        }
        costs.push_back(cost);
    }
    return costs;
}


void Tracker3D::trackInit(std::vector<Cluster> &objects,double_t time_){
    time=time_;
    for (auto object:objects) {
        tracks.insert(std::pair<int,Track3d>(trackID, newTrack(object)));
    }
    init= true;
}


void Tracker3D::update(std::vector<Cluster> &objects) {
    std::vector<std::vector<float>> cost_matrix= Iou_cost(objects);
    std::vector<int> detIds;
    std::vector<int> takIds;
    for (auto track:tracks) {
        takIds.push_back(track.first);
    }
    if(cost_matrix.empty()){
        std::cout<<"cost matrix is empty!"<<std::endl;
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
    for (int i = 0; i <objects.size() ; ++i) {
        auto iter=std::find(detIds.begin(),detIds.end(),i);
        if(iter>=detIds.end()){
            //新出现的目标未有trace对应 需要新建
            auto new_track=newTrack(objects[i]);//新建一个trace
            tracks.insert(std::pair<int,Track3d>(new_track.ID, new_track));
        }else{
            // 有对应trace匹配上
            int cur_trackID=takIds.at(iter-detIds.begin());
            cv::Mat measurement=cv::Mat::zeros(measures_dim,1,CV_32F);
            for (int row=0;row<objects[i].bbox3d.size();row++) {
                measurement.at<float>(row)=objects[i].bbox3d[row];
            }
            tracks.at(cur_trackID).kf.kalmanUpdate(measurement);
            //更新track对应的点云并保存为历史信息
            tracks.at(cur_trackID).historyPointClouds.push_back(tracks.at(cur_trackID).pointCloud);
            tracks.at(cur_trackID).pointCloud=objects[i].pointCloud;
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
            tracks[cur_trackId].no_match++;
        }else{
            tracks[cur_trackId].no_match=0;
        }
        checkState(cur_trackId);
    }
}

std::vector<fusion_perception::Track> Tracker3D::reportTracks() {
    //执行 对应Tracks的预测
//    predict();

    std::vector<fusion_perception::Track> tracks3d;
    for (auto track:tracks) {
        fusion_perception::Track track3d;
        track3d.classID=track.second.classID;
        track3d.age=track.second.age;
        track3d.trackID=track.second.ID;
        track3d.position.x,track3d.position.y,track3d.position.z  = track.second.state.at<float>(0),track.second.state.at<float>(1),track.second.state.at<float>(2);
        track3d.size.x,track3d.size.y,track3d.size.z  = track.second.state.at<float>(3),track.second.state.at<float>(4),track.second.state.at<float>(5);
        track3d.velocity.x,track3d.velocity.y,track3d.velocity.z  = track.second.state.at<float>(6),track.second.state.at<float>(7),track.second.state.at<float>(8);
        pcl::toROSMsg(track.second.pointCloud,track3d.pointCloud);
        for (auto cloud:track.second.historyPointClouds) {
            sensor_msgs::PointCloud2 history_cloud;
            pcl::toROSMsg(cloud,history_cloud);
            track3d.historyPointCloud.push_back(history_cloud);
        }
        tracks3d.push_back(track3d);
    }
    return tracks3d;
}
bool Tracker3D::reportInit() {
    return init;
}
std::vector<Track3d> Tracker3D::reportResults() {
    std::vector<Track3d> result;
    for (auto track:tracks) {
//        std::cout<<"state:"<<track.first<<" "<<track.second.age<<" "<<std::endl;
//        for (int i=0;i<track.second.state.rows;i++){
//            std::cout<<track.second.state.at<float>(i)<<" ";
//        }
//        std::cout<<"points_size:"<<track.second.pointCloud.points.size()<<std::endl;
//        std::cout<<std::endl;
        if (track.second.enable==true)
            result.push_back(track.second);
    }
    return result;
}
double_t Tracker3D::reportTime() {
    return time;
}