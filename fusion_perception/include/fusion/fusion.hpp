//
// Created by mzc on 2021/4/19.
//
#pragma once

#include "agnes.hpp"
#include "mkn.hpp"
#include "cstdlib"
#include "time.h"
#include "projection.hpp"
#include "geometry_msgs/Vector3.h"
class Fusion{
private:
    MTMSolver mtmSolver;//0-1 背包问题求解器
    AGNES agnes{0.1,100};
    Projection projection;
    std::vector<Track3d> original3d;
    std::vector<Track2d> original2d;
    std::vector<int> solve_match;
    int m,n;//m :objects;n:clusters
    float  mini_cost;
    std::vector<std::vector<float>> cost_base;
    std::vector<int> feed_random();
    std::tuple<float ,std::vector<int>> mengteSlove( std::vector<int> way);
    fusion_perception::Frame frame;
    float IouCalculate(cv::Rect rect1,cv::Rect rect2);
    float velCalculate(Eigen::Vector3f v1,Eigen::Vector3f v2);
    std::vector<std::vector<float>> costCalculate(std::vector<Reflect> &r2d,std::vector<Reflect> &r3d);
    bool trackManage(Track3d t3d);
    void deleteTrackMap();
    int mini_id=INT32_MAX;
    std::map<int,int> trakID_map;

public:
    void setInput(std::vector<Track3d> &points,std::vector<Track2d> &images);
    void fusionWithMengte(int feed_num);
    std::vector<DataPoint> fusionWithANGES();
    void fusionAgnesInit(cv::Size size,int camera_model,float thres,Eigen::Matrix4f extraTwb);
    void fusionHungarian(cv::Size size,int camera_model,float thres,Eigen::Matrix4f extraTwb);
    fusion_perception::Frame reportResult();
};
void Fusion::setInput(std::vector<Track3d> &points, std::vector<Track2d> &images) {
    original2d=images;
    original3d=points;
    m=original2d.size();
    n=original3d.size();
    frame.tracks.clear();
    frame.others.clear();
}

bool Fusion::trackManage(Track3d t3d) {
    int cur_id=t3d.ID;
    Eigen::Vector3f  vel;
    vel={t3d.state.at<float>(6),t3d.state.at<float>(7),t3d.state.at<float>(8)};
    if(vel.norm()>0.2)
        trakID_map[cur_id]++;
    if(trakID_map[cur_id]>2)
        return true;
}

void Fusion::deleteTrackMap() {
    for (auto m:trakID_map) {
        if (m.first<mini_id)
            trakID_map.erase(m.first);
    }
}
/**********************************
 * Mengta method
 *
 *********************************/
std::vector<int> Fusion::feed_random() {
    std::vector<int> random_solve;
    std::srand((unsigned)std::time(NULL));
    for (auto &feed:random_solve) {
        feed=std::rand()%(m+1);//生成0-m的随机数
    }
    return random_solve;
}

std::tuple<float ,std::vector<int>> Fusion::mengteSlove(std::vector<int> way) {
    float final_cost;
    std::vector<std::vector<float>> dis_cost(m,std::vector<float>(m,0));
    Eigen::MatrixXf cost_matrix(m,n);
    Eigen::MatrixXf para(n,m);
    std::vector<Ele> used3d;

    for (int i = 0; i < m; ++i) {
        Ele ele_new;
        for (int j=0;j<n;j++) {
            if(way[j]==i){
                ele_new.points_num+=used3d.at(j).points_num;
                ele_new.center_point+=used3d.at(j).center_point;
                para(j,i)=1.0;
            } else{
                para(j,i)=0.0;
            }
        }
        ele_new.center_point/=ele_new.points_num;
        float max_dis=0;
        for (int j=0;j<n;j++) {
            if (way[j]==i){
                float dis=(used3d[j].center_point-ele_new.center_point).norm();
                max_dis= max_dis>=dis ? max_dis : dis;
                dis_cost[i][j]=dis;
            }
        }
        for (int j=0;j<n;j++) {
            if (way[j]==i){
                dis_cost[i][j]/=max_dis;
            }
        }
    }
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            cost_matrix(i,j)=cost_base[i][j]+dis_cost[i][j];
        }
    }
    auto cost=para*cost_matrix;
    final_cost=cost.sum();
    return std::tuple<float ,std::vector<int>>{final_cost,way};
}

void Fusion::fusionWithMengte(int feed_num) {
    cost_base.resize(m,std::vector<float>(n,0));
    long long int max_size= pow(m+1,n);
    std::vector<DataPoint> combined3d;
    if (feed_num> max_size)
        feed_num=max_size;

    agnes.setOriginData(original3d);
    agnes.startAgnes();
    //层次聚类获取一个3D points的初值
    combined3d=agnes.reportCluster();
//    auto reflects = projection.projectionToPixs(combined3d);
    //base cost calculate
    for (int i=0;i<m;i++){
        Reflect object;
        object.rect={original2d[i].state.at<float>(0),original2d[i].state.at<float>(1),original2d[i].state.at<float>(2),original2d[i].state.at<float>(3)};
        object.vel={original2d[i].state.at<float>(4),original2d[i].state.at<float>(5),std::min(original2d[i].state.at<float>(6),original2d[i].state.at<float>(7))};
        for (int j = 0; j < n; ++j) {
           float Iou_cost=(object.rect & combined3d.at(j).reflect.rect).area() / (object.rect | combined3d.at(j).reflect.rect).area();
           float vel_cost=object.vel.dot(combined3d.at(j).reflect.vel)/(object.vel.norm()*combined3d.at(j).reflect.vel.norm());
           cost_base.at(i).at(j)=Iou_cost+vel_cost;
        }
    }
    while (feed_num){
        feed_num--;
        auto cur_way = mengteSlove(feed_random());
        if(std::get<0>(cur_way)<mini_cost)
            mini_cost=std::get<0>(cur_way);
            solve_match=std::get<1>(cur_way);
    }
    //动态规划&分支定界算法求解
//    std::vector<int> profits,  weights,  capacities;
//    std::vector<Ele> init;
//    for (auto object:original2d) {
//        cv::Rect2f rectO{object.state.at<float>(0),object.state.at<float>(1),object.state.at<float>(2),object.state.at<float>(2)};
//        capacities.push_back(rectO.area());
//        for (auto ref:reflects) {
//
//        }
//    }
//    mtmSolver.setInput(profits,weights,capacities,init);
//    mtmSolver.solve();
}

/**********************************
 * ANGES method
 *
 *********************************/
void Fusion::fusionAgnesInit(cv::Size size, int camera_model,float thres,Eigen::Matrix4f extraTwb ) {
    agnes.projection.setModel(camera_model);
    agnes.projection.setImageSize(size);
    agnes.setParam(thres,0);
    agnes.projection.setExtraTwb(extraTwb);

}

std::vector<DataPoint> Fusion::fusionWithANGES() {
    std::vector<DataPoint> res;
    std::vector<Reflect> reflects2d;
    for (auto obj:original2d){
        Reflect temp;
        temp.rect={obj.state.at<float>(0),obj.state.at<float>(1),obj.state.at<float>(2),obj.state.at<float>(3)};
        temp.vel={obj.state.at<float>(4),obj.state.at<float>(5),-(abs(obj.state.at<float>(6))> std::abs(obj.state.at<float>(7)) ? obj.state.at<float>(7) :obj.state.at<float>(6))};
        temp.ID=obj.ID;
        temp.classID=obj.classID;
        reflects2d.push_back(temp);
    }
    agnes.setOriginData(original3d);
    agnes.setReflects(reflects2d);
    agnes.startAgnes();
    res= agnes.reportCluster();
    std::cout<<original3d.size()<<" "<<res.size()<<std::endl;
    return res;
}


/********************************
 * Hungarian method
 *
 * ***************************/
float Fusion::velCalculate(Eigen::Vector3f v1, Eigen::Vector3f v2) {
    float cosT = v1.dot(v2)/(v1.norm()*v2.norm());
    return cosT;
}

float Fusion::IouCalculate(cv::Rect rect1, cv::Rect rect2) {
    float Iou=(rect1 & rect2).area()*1.0/(rect1 | rect2).area();
    return Iou;
}

std::vector<std::vector<float>> Fusion::costCalculate(std::vector<Reflect> &r2ds, std::vector<Reflect> &r3ds) {
    std::vector<std::vector<float>> costs;
    for(auto r2d:r2ds){
        std::vector<float> cost;
        for(auto r3d : r3ds){
            float ele_Iou = IouCalculate(r2d.rect, r3d.rect);
            float ele_vel= velCalculate(r2d.vel,r3d.vel);
//            float ele=0.5*ele_vel+ele_Iou;
            float ele=ele_Iou;
            ele=1-ele;
            cost.push_back(ele);
        }
        costs.push_back(cost);
    }
    return costs;
}

void Fusion::fusionHungarian(cv::Size size, int camera_model, float thres,Eigen::Matrix4f extraTwb) {
    frame.others.clear();
    frame.tracks.clear();
    projection.setModel(camera_model);
    projection.setImageSize(size);
    projection.setExtraTwb(extraTwb);
    std::vector<Reflect> reflects2d;
    cv::Mat img_show(size,CV_8UC3,cv::Scalar(255,255,255));
    for (auto obj:original2d){
        Reflect temp;
        temp.rect={obj.state.at<float>(0),obj.state.at<float>(1),obj.state.at<float>(2),obj.state.at<float>(3)};
        temp.vel={obj.state.at<float>(4),obj.state.at<float>(5),-(abs(obj.state.at<float>(6))> std::abs(obj.state.at<float>(7)) ? obj.state.at<float>(7) :obj.state.at<float>(6))};
        temp.ID=obj.ID;
        temp.classID=obj.classID;
        reflects2d.push_back(temp);
        cv::rectangle(img_show, temp.rect, cv::Scalar(0x27, 0xC1, 0x36), 2);//绘制矩形框

    }
    std::vector<Reflect> reflects3d;
    for (auto obj:original3d) {
        Reflect temp;
        temp=projection.projectionToPix(obj);
        if(temp.inner_image== true)
        {
            reflects3d.push_back(temp);
            cv::rectangle(img_show, temp.rect, cv::Scalar(0x00, 0xC1, 0xFF), 2);//绘制矩形框
        }
    }
    cv::imshow("projection",img_show);
    std::vector<std::vector<float>> cost_matrix = costCalculate(reflects2d,reflects3d);
    std::vector<int> r3dIds;
    HungarianAlgorithm matcher;
    if(cost_matrix.empty()){
//        std::cout<<"cost matrix is empty!"<<std::endl;
    }else{
//        std::cout<<"cost_matrix:"<<std::endl;
//        for (auto line:cost_matrix) {
//            for (auto ele:line) {
//                std::cout<<ele<<" ";
//            }
//            std::cout<<std::endl;
//        }
        matcher.Solve(cost_matrix,r3dIds);
    }
    for (int i = 0; i < reflects2d.size(); ++i) {
        fusion_perception::Track track;
        int r3dIdx=-1;
        if(r3dIds.size()>0)
            r3dIdx=r3dIds[i];
//        std::cout<<"index:"<<r3dIdx<<std::endl;
        if(r3dIdx>=0&&cost_matrix[i][r3dIdx]<thres)
        {
            //match success
            track.classID=original2d[i].classID;
            track.age=original2d[i].age;
            track.trackID=original2d[i].ID;
            track.classname=original2d[i].classname;
            auto tr3d=original3d[r3dIdx];
            original3d[r3dIdx].enable= false;
            pcl::toROSMsg(tr3d.pointCloud,track.pointCloud);
            track.position.x=tr3d.state.at<float>(0);track.position.y=tr3d.state.at<float>(1);track.position.z=tr3d.state.at<float>(2);
            track.size.x=tr3d.state.at<float>(3);track.size.y=tr3d.state.at<float>(4);track.size.z=tr3d.state.at<float>(5);
            track.velocity.x=tr3d.state.at<float>(6);track.velocity.y=tr3d.state.at<float>(7);track.velocity.z=tr3d.state.at<float>(8);
            for (auto state:tr3d.history_states) {
                geometry_msgs::Vector3 position;
                position.x=state.at<float>(0),position.y=state.at<float>(1),position.z=state.at<float>(2);
                track.trace.push_back(position);
                Eigen::Vector3f vel;
                vel={state.at<float>(6),state.at<float>(7),state.at<float>(8)};
                track.historyVelNorm.push_back(vel.norm());
            }
            for (auto points:tr3d.historyPointClouds) {
                sensor_msgs::PointCloud2 pointCloud2;
                pcl::toROSMsg(points,pointCloud2);
                track.historyPointCloud.push_back(pointCloud2);
            }
        }else{
           // match failed

        }
        frame.tracks.push_back(track);
    }

    for (auto tr3d:original3d) {
        if(tr3d.ID<mini_id)
            mini_id=tr3d.ID;
        if (tr3d.enable==false)
            return;
        if(!trackManage(tr3d))
            return;
        fusion_perception::Track track;
        track.classID=-1;
        track.classname="unknown";
        track.age=tr3d.age;
        track.trackID=tr3d.ID;
        track.position.x=tr3d.state.at<float>(0);track.position.y=tr3d.state.at<float>(1);track.position.z=tr3d.state.at<float>(2);
        track.size.x=tr3d.state.at<float>(3);track.size.y=tr3d.state.at<float>(4);track.size.z=tr3d.state.at<float>(5);
        track.velocity.x=tr3d.state.at<float>(6);track.velocity.y=tr3d.state.at<float>(7);track.velocity.z=tr3d.state.at<float>(8);
//        Eigen::Vector3f  vel={track.velocity.x,track.velocity.y,track.velocity.z};
//        float norm= vel.norm();
//        if(norm<0.1)
//            return;
        for (auto state:tr3d.history_states) {
            geometry_msgs::Vector3 position;
            position.x=state.at<float>(0),position.y=state.at<float>(1),position.z=state.at<float>(2);
            track.trace.push_back(position);
            Eigen::Vector3f vel;
            vel={state.at<float>(6),state.at<float>(7),state.at<float>(8)};
            track.historyVelNorm.push_back(vel.norm());
        }
        for (auto points:tr3d.historyPointClouds){
            sensor_msgs::PointCloud2 pointCloud2;
            pcl::toROSMsg(points,pointCloud2);
            track.historyPointCloud.push_back(pointCloud2);
        }
        pcl::toROSMsg(tr3d.pointCloud,track.pointCloud);
        frame.tracks.push_back(track);
    }
    deleteTrackMap();
}

//report
fusion_perception::Frame Fusion::reportResult(){
    if(!frame.tracks.empty())
        std::cout<<"report frame:"<<std::endl;
    for(auto tr3d:frame.tracks){
        std::cout<<tr3d.classID<<" "<<tr3d.trackID<<" "<<tr3d.age<<std::endl;
    }
    return  frame;
}
