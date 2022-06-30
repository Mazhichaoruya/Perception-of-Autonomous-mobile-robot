#pragma once
#include "iostream"
#include "vector"
#include "mkn.hpp"
#include "fusion_perception/Track.h"
#include "agnes.hpp"
#include "fusion.hpp"


class Combine{
private:

   AGNES agnes{1.0,10};
   std::vector<Track3d> original_targets;
   std::vector<Track3d> combined_targets;
   std::vector<std::vector<int>> targetsID;
    std::vector<Reflect> reflects2d;

public:
    void setTracks(std::vector<Track3d> &tracks);
    void setReflects(std::vector<Reflect> r2ds);
    void combine();
    std::vector<Track3d> reportTargets();
    std::vector<std::vector<int>> reportIDs();

};
void Combine::setReflects(std::vector<Reflect> r2ds) {
    agnes.setParam(0.4,original_targets.size());
    agnes.setReflects(r2ds);
}
void Combine::setTracks(std::vector<Track3d> &tracks) {
    original_targets=tracks;
    targetsID.clear();
    combined_targets.clear();
}
void Combine::combine() {
    agnes.setOriginData(original_targets);
    agnes.startAgnes();
    auto cluster_data=agnes.reportCluster();

    //以加权平均为原则进行合并
//    for (auto cluster:cluster_data) {
//        Track3d target;
//        std::vector<int> targetID;
//        for (auto t:cluster) {
//            targetID.push_back(t.ID);
//            target.pointCloud+=t.pointCloud;
//            target.classID=t.classID;
//        }
//        for (auto t:cluster) {
//            target.ID=t.clusterID;
//            target.state+=t.state*t.pointCloud.size()/target.pointCloud.points.size();
//        }
//        targetsID.push_back(targetID);
//        combined_targets.push_back(target);
//    }
}
std::vector<Track3d> Combine::reportTargets() {
    return combined_targets;
}
std::vector<std::vector<int>> Combine::reportIDs() {
    return  targetsID;
}