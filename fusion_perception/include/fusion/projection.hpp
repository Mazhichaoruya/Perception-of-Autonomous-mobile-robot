#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "pcl/point_types.h"
#include "tracker/tracker3d.hpp"

struct Reflect{
    int ID;
    int classID;
    cv::Rect2f rect;
    Eigen::Vector3f  vel;
    bool inner_image=true;
    int point_num;
};

class Projection{
private:
    Eigen::Matrix3f inner;
    Eigen::Matrix4f extraTwb;
    Eigen::Matrix4f extraTbc;
    Eigen::Matrix4f extraTbw;
    Eigen::Matrix4f extraTcb;
    int camera_model;
    cv::Size imageSize;
    cv::Rect2f imageRect;
    bool enable=false;
    Eigen::Vector3f pointConvertBall(Eigen::Vector3f point);
public:
    std::vector<Reflect> projectionToPixs(std::vector<Track3d> &tracks);
    Eigen::Vector3f pointConvertToPix(Eigen::Vector3f point);
    Eigen::Vector3f pointConvertCam(Eigen::Vector3f point);

    Reflect projectionToPix(Track3d &track);
    Projection();
    void setExtraTwb(Eigen::Matrix4f extra);
    void setModel(int model);
    void setImageSize(cv::Size size);
};
Projection::Projection() {
    extraTbc<<0,0,1,0.05,
            -1,0,0,0.0,
            0,-1,0,0.02,
            0,0,0,1;
//    std::cout<<"extraTC:"<<std::endl<<extraTbc<<std::endl;
    extraTcb=extraTbc.inverse();
}

void Projection::setImageSize(cv::Size size) {
    imageSize=size;
    imageRect={0,0,float(imageSize.width),float(imageSize.height)};
}

void Projection::setModel(int model) {
    camera_model=model;
    if(model==0){
        inner<<392.274791,0.000000,297.758189,
                0.000000 ,394.329639, 255.158130,
                0.000000, 0.000000 ,1.000000;
    }else if(model==1){
//        inner<<203.718327,0.000000 ,319.500000
//        ,0.000000 ,203.718327 ,239.500000
//        ,0.000000 ,0.000000 ,1.000000;
        inner<<392.274791,0.000000,320,
                0.000000 ,394.329639, 240,
                0.000000, 0.000000 ,1.000000;
    }
    extraTwb=Eigen::Matrix4f::Identity();
    extraTbw=extraTwb;
}
//鱼眼相机模型需要首先要转化到球面上
Eigen::Vector3f Projection::pointConvertBall(Eigen::Vector3f point){
    Eigen::Vector3f point_3d;
    point_3d=point/point(2);
    if (camera_model==0)
        return point_3d;
//    std::cout<<"point_3d:"<<std::endl<<point_3d<<std::endl;
    double r=std::sqrt(point_3d(0)*point_3d(0)+point_3d(1)*point_3d(1));
    float tha=std::atan(r);
    float k1=0.1,k2=0.2;
    float thd = tha/*+k1* pow(tha,2)+k2*pow(tha,4)*/;
    Eigen::Vector3f point_sur;
    point_sur=thd/r*point_3d;
    Eigen::Vector3f res;
//    std::cout<<"point_surf:"<<std::endl<<point_sur<<std::endl;
    float x=point_sur(0)/*+point_3d.x()*point_sur.y()*/;
    float y= point_sur(1);
    res<<x,y,1;
//    std::cout<<"res:"<<std::endl<<res<<std::endl;
    return res;
}
void Projection::setExtraTwb(Eigen::Matrix4f extra) {
    extraTwb=extra;
    extraTbw=extra.inverse();
}
//点云转化为平面像素
Eigen::Vector3f Projection::pointConvertToPix(Eigen::Vector3f point){

    Eigen::Vector4f point_world,point_body,point_cam4;
    Eigen::Vector3f point_cam,pix;
    point_world<<point.x(),point.y(),point.z(),1;
//    std::cout<<"point_world:"<<std::endl<<point_world<<std::endl;
    point_body=extraTbw*point_world;
//    std::cout<<"point_body:"<<std::endl<<point_body<<std::endl;
    point_cam4=extraTcb*point_body;
//    std::cout<<"point_cam4:"<<std::endl<<point_cam4<<std::endl;
    point_cam<<point_cam4(0),point_cam4(1),point_cam4(2);
//    point_cam<<-point_cam4(1),-point_cam4(2),point_cam4(0);
//    point_cam<<-point_cal(1),-point_cal(2),point_cal(0);
    auto point_ball= pointConvertBall(point_cam);
//    std::cout<<"point_cam:"<<std::endl<<point_cam<<std::endl;
    pix=inner*point_ball;
    pix(2)=point_cam(2);
//    std::cout<<"point_pix:"<<std::endl<<pix<<std::endl;
    return pix;
}
//点云转化为相机坐标系
Eigen::Vector3f Projection::pointConvertCam(Eigen::Vector3f point){

    Eigen::Vector4f point_world,point_body,point_cam4;
    Eigen::Vector3f point_cam,pix;
    point_world<<point.x(),point.y(),point.z(),1;
    point_body=extraTbw*point_world;
    point_cam4=extraTcb*point_body;
    point_cam<<point_cam4(0),point_cam4(1),point_cam4(2);
    return point_cam;

}

Reflect Projection::projectionToPix(Track3d &t3d){
    Reflect reflect;
    reflect.ID=t3d.ID;
    reflect.classID=t3d.classID;
    reflect.point_num=t3d.pointCloud.points.size();
    reflect.inner_image= true;
    Eigen::Vector3f vet_max,vet_min;
    Eigen::Vector3f center,size,velocity;
    center<<t3d.state.at<float>(0),t3d.state.at<float>(1),t3d.state.at<float>(2);
    size<<t3d.state.at<float>(3),t3d.state.at<float>(4),t3d.state.at<float>(5);
    velocity<<t3d.state.at<float>(6),t3d.state.at<float>(7),t3d.state.at<float>(8);
    vet_max=center+size/2;
    vet_min=center-size/2;
//    std::cout<<"max:"<<vet_max.transpose()<<std::endl;
//    std::cout<<"min:"<<vet_min.transpose()<<std::endl;
    std::array<Eigen::Vector3f,8> vets,rects;
    for (int i=0;i<vets.size();i++) {
        int labelz=i&0x04;
        int labely=i&0x02;
        int labelx=i&0x01;
        vets[i].x()=labelx<=0?vet_min.x():vet_max.x();
        vets[i].y()=labely<=0?vet_min.y():vet_max.y();
        vets[i].z()=labelz<=0?vet_min.z():vet_max.z();
    }
    for (int i=0;i<vets.size();i++) {
//        std::cout<<"3d:"<<vets[i].transpose()<<std::endl;
        rects[i]= pointConvertToPix(vets[i]);
//        std::cout<<"2d:"<<rects[i].transpose()<<std::endl;
    }
//    auto rect_max= pointConvertToPix(vet_max);
//    auto rect_min= pointConvertToPix(vet_min);
    auto vel_img = pointConvertCam(velocity);
//    float r[2]={1000,1000};float s[2];float e[2]={-1000,-1000};
    Eigen::Vector3f r,s,e;
    r={1000,1000,0};
    e={-1000,-1000,0};
//    for (int i=0;i<2;i++) {
//        if (rect_max[i]>rect_min[i])
//            r[i]=rect_min[i];
//        else
//            r[i]=rect_max[i];
//        s[i]=std::abs(rect_max[i]-rect_min[i]);
//        e[i]=r[i]+s[i];
//    }
    for(auto rect:rects){
        if(rect[2]<0){
            reflect.inner_image= false;
        }else{
//            std::cout<<"add"<<std::endl;
        }
        for(int i = 0; i < 2; ++i){
            if(r[i]>rect[i])
                r[i]=rect[i];
            if(e[i]<rect[i])
                e[i]=rect[i];
        }
    }
//    std::cout<<"final_min:"<<r.transpose()<<std::endl;
//    std::cout<<"final_max:"<<e.transpose()<<std::endl;
    s=e-r;
    reflect.rect.x=r[0];
    reflect.rect.y=r[1];
    reflect.rect.width=s[0];
    reflect.rect.height=s[1];
    auto rectI=imageRect & reflect.rect;
    if (rectI.area()<=0)
        reflect.inner_image= false;
    reflect.vel=vel_img;
    return reflect;
}

std::vector<Reflect> Projection::projectionToPixs(std::vector<Track3d> &tracks) {
    std::vector<Reflect> reflects;
    for (auto t3d:tracks) {
        auto reflect = projectionToPix(t3d);
        if (reflect.inner_image)
            reflects.push_back(reflect);
    }
    return  reflects;
}