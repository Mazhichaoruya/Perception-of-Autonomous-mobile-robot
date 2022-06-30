#pragma once
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tracker/tracker3d.hpp"
class Visualization3D{
private:
    visualization_msgs::Marker center_points,Boxs,classname,arrow;
    visualization_msgs::MarkerArray classnames,arrows;
    void clearMarker(visualization_msgs::Marker &mk);
public:
    void initMarker();
    void setVisualization(std::vector<Track3d> tracks,std_msgs::Header cur_head);
    void setVisualization(fusion_perception::Frame frame ,std_msgs::Header cur_head);

    visualization_msgs::Marker reportCenters();
    visualization_msgs::MarkerArray reportArrows();
    visualization_msgs::MarkerArray reportClassnames();
};


void Visualization3D::initMarker() {
    arrow.ns=classname.ns=center_points.ns = Boxs.ns  = "Detected Bboxs";
//    arrow.action=classname.action=center_points.action = Boxs.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w=classname.pose.orientation.w=center_points.pose.orientation.w = Boxs.pose.orientation.w  = 1.0;
    center_points.type=visualization_msgs::Marker::POINTS;
    Boxs.type=visualization_msgs::Marker::LINE_LIST;
    classname.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    arrow.type=visualization_msgs::Marker::ARROW;
    center_points.scale.x=0.2;
    center_points.scale.y=0.2;//尺寸
    Boxs.scale.x=0.02;
    classname.scale.z=0.2;
    arrow.scale.x=0.03;
    arrow.scale.y=0.06;
    arrow.scale.z=0.02;
    center_points.color.g=1.0;
    center_points.color.a=1.0;//颜色
    Boxs.color.b=1.0f;
    Boxs.color.a=1.0;
    classname.color.r=1.0;
    classname.color.a=1.0;
    arrow.color.r=1.0;
    arrow.color.b=1.0;
    arrow.color.a=1.0;
}

void Visualization3D::setVisualization(std::vector<Track3d> tracks,std_msgs::Header cur_head) {
    center_points.points.clear();
    classnames.markers.clear();
    arrows.markers.clear();
    arrow.points.clear();
    arrow.header=classname.header=center_points.header=cur_head;
    arrow.action=visualization_msgs::Marker::MODIFY;
//    center_points.action=visualization_msgs::Marker::ADD;
    classname.action=visualization_msgs::Marker::MODIFY;
    center_points.id=0;Boxs.id=1;arrow.id=2;classname.id=3;
    for (int i = 0; i < tracks.size(); ++i) {
        geometry_msgs::Point p,p1,p2;
        auto state=tracks[i].state;
        p.x=state.at<float>(0);p.y=state.at<float>(1);p.z=state.at<float>(2);
        p1.x=p.x+state.at<float>(3)/2;p1.y=p.y+state.at<float>(4)/2;p1.y=p.y+state.at<float>(5)/2;
        p2.x=p.x+state.at<float>(6);p2.y=p.y+state.at<float>(7);p2.z=p.z+state.at<float>(8);
        center_points.points.push_back(p);
        classname.pose.position=p1;
        classname.text=tracks[i].classname+std::to_string(tracks[i].ID)+":"+std::to_string(tracks[i].age);
        classnames.markers.push_back(classname);
        arrow.points.push_back(p);arrow.points.push_back(p2);
        arrows.markers.push_back(arrow);
        arrow.id+=2;
        classname.id=arrow.id+1;
    }
}

void Visualization3D::setVisualization(fusion_perception::Frame frame, std_msgs::Header cur_head) {
    center_points.points.clear();
    classnames.markers.clear();
    arrows.markers.clear();
    arrow.points.clear();
    arrow.header=classname.header=center_points.header=cur_head;
    arrow.action=visualization_msgs::Marker::ADD;
    center_points.action=visualization_msgs::Marker::ADD;
    classname.action=visualization_msgs::Marker::ADD;
    center_points.id=0;Boxs.id=1;arrow.id=2;classname.id=3;
    if(frame.tracks.empty()){
        clearMarker(classname);
        clearMarker(center_points);
        clearMarker(arrow);
    }
    for(auto track:frame.tracks){
        geometry_msgs::Point p,p1,p2;
        p.x=track.position.x;p.y=track.position.y;p.z=track.position.z;
        p1.x=p.x+track.size.x/2;p1.y=p.y+track.size.y/2;p1.y=p.y+track.size.z/2;
        p2.x=p.x+track.velocity.x;p2.y=p.y+track.velocity.y;p2.z=p.z+track.velocity.z;
        center_points.points.push_back(p);
        classname.pose.position=p1;
        classname.text=std::to_string(track.classID)+"--"+std::to_string(track.trackID)+":"+std::to_string(track.age);
        classnames.markers.push_back(classname);
        arrow.points.push_back(p);arrow.points.push_back(p2);
        arrows.markers.push_back(arrow);
        arrow.id+=2;
        classname.id=arrow.id+1;
    }
}

void Visualization3D::clearMarker(visualization_msgs::Marker &mk) {
    mk.action=visualization_msgs::Marker::DELETE;
}

visualization_msgs::MarkerArray Visualization3D::reportArrows() {
    return arrows;
}

visualization_msgs::Marker Visualization3D::reportCenters() {
    return center_points;
}

visualization_msgs::MarkerArray Visualization3D::reportClassnames() {
    return classnames;
}
