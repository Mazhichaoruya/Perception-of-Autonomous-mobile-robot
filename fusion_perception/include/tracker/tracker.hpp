#pragma once

//
// Created by mzc on 2021/2/18.
//
#include "kalman/kalman.hpp"
#include "matcher/hungarian.hpp"
#include "map"


struct Track{
    int ID;//track ID
    int no_match;//丢失匹配时间
    bool enable;//有效标志位 --true有效（report均为有效track ）
    int classID=-1;//类别编号
    std::string classname;//类别名称
    int age;//已创建周期
    cv::Mat state;
    std::vector<cv::Mat> history_states;
};
template <typename  TrackT>
class Tracker{

protected:
    std::map<int,TrackT> tracks;
    float threshd;//1-thresh
    int trackID;
    int death_period,birth_period;//生存与死亡周期
    HungarianAlgorithm matcher;
    bool init= false;
    double_t time;
    double dt=0.1;
public:
    virtual void checkState(int trackId);
    virtual void setParam(int birth,int death,float th,float dt_);
    virtual void predict();

};
template <typename  TrackT>
void Tracker<TrackT>::checkState(int trackId) {
    if (tracks.at(trackId).age>=birth_period){
        tracks.at(trackId).enable= true;
    }
    if (tracks.at(trackId).no_match>0){
        if(tracks.at(trackId).no_match>=death_period)
            tracks.erase(trackId);
        else
            tracks.at(trackId).enable= false;
    }
}
template <typename  TrackT>
void Tracker<TrackT>::setParam(int birth, int death, float th,float dt_) {
    birth_period=birth;
    death_period=death;
    threshd=th;
    dt=dt_;
}
template <typename  TrackT>
void Tracker<TrackT>::predict() {
    for (auto &track:tracks) {
        track.second.kf.kalmanPredict();
        track.second.state=track.second.kf.getPreState();
        track.second.history_states.push_back(track.second.state);
        track.second.age++;
    }
    time+=dt;
}