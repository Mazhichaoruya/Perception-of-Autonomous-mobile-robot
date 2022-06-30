#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "pcl/point_types.h"
#include "tracker/tracker3d.hpp"
#include "projection.hpp"


struct DataPoint:public Track3d{
    int clusterID;
    Reflect reflect;
};

class AGNES
{

public:
    AGNES(double ThreshordDis, int MaxClu) :ThreshordDis(ThreshordDis), MaxClu(MaxClu) {
    }
    ~AGNES() {
    }
    std::vector<DataPoint> reportCluster();
    void startAgnes(); //算法核心
    void setOriginData(std::vector<Track3d> &data);//获取数据
    void setParam(double threshordDis, int maxClu);
    void setReflects(std::vector<Reflect> r2ds);
    Projection projection;

private:
    std::vector<Reflect> reflects2d;
    double getDistance(DataPoint &point, DataPoint &point2); //得到两个数据之间的距离
    double getCDistance(std::vector<DataPoint> C, std::vector<DataPoint> C2); //得到两个聚类之间的最小距离
    double getAVGDistance(std::vector<DataPoint> &C, std::vector<DataPoint> &C2);//得到两个聚类之间的平均距离
    std::vector<DataPoint> DataBase; //保存所有数据
    std::vector<std::vector<DataPoint>> CluData; //保存所有簇类数据
    std::vector<std::vector<float>> DataMap; //保存两两聚类之间的距离 此处包含距离和运动方向
    float calculateVel(cv::Mat d1,cv::Mat d2);
    float calculatePos(cv::Mat d1,cv::Mat d2);
    float calculateIou(Reflect r1,Reflect r2);
    DataPoint mergeData(std::vector<DataPoint> clus,int index);
    cv::Mat mergeState(std::vector<DataPoint> clus);
    Reflect mergeProjection(std::vector<DataPoint> clus);
    std::array<float ,6> calculateBbox(pcl::PointCloud<pcl::PointXYZI> &clouds);

    int model=0; //0 非监督 1 监督
    float ThreshordDis;
    float CurMinDis;
    int MaxClu; //最终获得聚类数量
    int  p; //存储最初的聚类的个数
};


std::array<float ,6>  AGNES::calculateBbox(pcl::PointCloud<pcl::PointXYZI> &clouds){
    int size=clouds.points.size();
    float max_x=INT32_MIN,max_y=INT32_MIN,max_z=INT32_MIN,min_x=INT32_MAX,min_y=INT32_MAX,min_z=INT32_MAX;
    float sum_x=0,sum_y=0,sum_z=0;
    for (auto p:clouds){
        if (max_x<p.x) max_x=p.x;
        if (max_y<p.y) max_y=p.y;
        if (max_z<p.z) max_z=p.z;
        if (min_x>p.x) min_x=p.x;
        if (min_y>p.y) min_y=p.y;
        if (min_z>p.z) min_z=p.z;
        sum_x+=p.x;
        sum_y+=p.y;
        sum_z+=p.z;
    }
    return std::array<float,6>{sum_x/size,sum_y/size,sum_z/size,max_x-min_x,max_y-min_y,max_z-min_z};
}


void AGNES::setParam(double threshordDis, int maxClu) {
    ThreshordDis=threshordDis;
    MaxClu=maxClu;
}
void AGNES::setReflects(std::vector<Reflect> r2ds) {
    model=1;
    reflects2d=r2ds;
}
float AGNES::calculateVel(cv::Mat d1, cv::Mat d2) {
    Eigen::Vector3f v1,v2;
    v1={d1.at<float>(6),d1.at<float>(7),d1.at<float>(8)};
    v2={d2.at<float>(6),d2.at<float>(7),d2.at<float>(8)};
    float cosT = v1.dot(v2)/(v1.norm()*v2.norm());
    return cosT;
}

float AGNES::calculatePos(cv::Mat d1, cv::Mat d2) {
    Eigen::Vector3f p1,p2,s1,s2;
    p1={d1.at<float>(0),d1.at<float>(1),d1.at<float>(2)};
    p2={d2.at<float>(0),d2.at<float>(1),d2.at<float>(2)};
    s1={d1.at<float>(3),d1.at<float>(4),d1.at<float>(5)};
    s2={d2.at<float>(3),d2.at<float>(4),d2.at<float>(5)};
    auto dis_center=(p2-p1).norm();
    auto dis_size=(s2+s1).maxCoeff();
    float dis=dis_center-dis_size;
    return dis;
}

float AGNES::calculateIou(Reflect r1, Reflect r2) {
    float Iou_cost=(r1.rect & r2.rect).area() / (r1.rect | r2.rect).area();
    if (Iou_cost<1e-3)
        return Iou_cost;
    float vel_cost=r1.vel.dot(r2.vel)/(r1.vel.norm()*r2.vel.norm());
    return  Iou_cost+vel_cost;
}

void AGNES::setOriginData(std::vector<Track3d> &data) {
    int index=0;
    CluData.clear();
    DataMap.clear();
    DataBase.clear();
    reflects2d.clear();

    for (auto d:data) {
        DataPoint dp;
        dp.clusterID=index;
        index++;
        dp.state=d.state;
        dp.reflect=projection.projectionToPix(d);
        dp.pointCloud=d.pointCloud;
        dp.ID=d.ID;
        DataBase.push_back(dp);
    }
    DataMap.resize(data.size(),std::vector<float>(data.size()));
}
void AGNES::startAgnes()
{
    //获得每个数据距离其他数据的距离
    for (int i = 0; i < DataBase.size(); i++)
    {

        std::vector<DataPoint> t;
        t.push_back(DataBase[i]);
        CluData.push_back(t);
    }
    //DataMap数组保存两个聚类之间的距离
    for (int i = 0; i < CluData.size(); i++)
    {

        std::vector<DataPoint> temp;
        for (int j =i+1; j < CluData.size(); j++)
        {

            double dis = getAVGDistance(CluData[i], CluData[j]);
            DataMap[i][j] = dis;
            DataMap[j][i] = dis;
        }
    }
    p = DataBase.size(); //设置聚类个数
    while (p > MaxClu)
    {

        double Temp = 9999;
        int Find_i = 0, Find_j = 0;
        //寻找最小值点
        for (int i = 0; i < p; i++)
        {

            for (int j = i+1; j < p; j++)
            {

                if (DataMap[i][j] < Temp)
                {

                    Temp = DataMap[i][j];
                    Find_i = i;
                    Find_j = j;
                }
            }
        }
        CurMinDis=Temp;
        if(CurMinDis>ThreshordDis){
            break;
        }
        int NewId = CluData[Find_i][0].clusterID; //获取当前簇的聚类号码
        //将寻找到的最小点的两个簇合并 A1+B1->A1
        //并将两个簇的簇类号码统一
        for (int j = 0; j < CluData[Find_j].size(); j++)
        {
            CluData[Find_j][j].clusterID=NewId;
            CluData[Find_i].push_back(CluData[Find_j][j]);
        }
        //重编号矩阵
        for (int i = Find_j + 1; i < p; i++)
        {

            for (int j = 0; j < CluData[i].size(); j++)
            {

                CluData[i][j].clusterID=CluData[i][j].clusterID- 1;
            }
        }
        //距离矩阵重置
        for (int i = Find_j; i < CluData.size()-1; i++)
        {

            CluData[i] = CluData[i + 1];
        }
        //删除DataMap矩阵 行列
        for (int i = 0; i < p; i++)
        {

            for (int j = Find_j; j < p-1; j++)
            {
                DataMap[i][j] = DataMap[i][j + 1];
            }
        }
        for (int i = Find_j; i < p-1; i++)
        {

            for (int j = 0; j < p; j++)
            {
                DataMap[i][j] = DataMap[i + 1][j];
            }
        }
        p = p - 1;
        //重新计算合并数据后的聚类与其他聚类之间的距离
        for (int i = 0; i < p; ++i)
        {
//            std::cout<<"i:"<<i<<std::endl;
            double dis = getAVGDistance(CluData[Find_i], CluData[i]);
            if (DataMap[Find_i][i] != dis)
            {

                DataMap[Find_i][i] = dis;
                DataMap[i][Find_i] = dis;
            }
        }
        //获取当前分组的最大距离值值
//        CurMinDis=0;
//        for (int i = 0; i < p; i++)
//        {
//            for (int j = 0; j < p; j++)
//            {
//                if (DataMap[i][j] <CurMinDis&&DataMap[i][j]>1e-7)
//                {
//                    CurMinDis = DataMap[i][j];
//                }
//            }
//        }
    }
}

//两个数据之间的距离
double AGNES::getDistance(DataPoint &point1, DataPoint &point2)
{
//    float  vel_dis= v1.dot(v2)/(v1.norm()*v2.norm());
//    float pos_dis= (p2-p1).norm();
//    return vel_dis+pos_dis;
}
cv::Mat AGNES::mergeState(std::vector<DataPoint> clus) {
    cv::Mat res;
    for (int i = 0; i < clus.size(); i++)
    {
        res+=clus[i].state;
        Eigen::Vector3f  temp={clus[i].state.at<float>(0),clus[i].state.at<float>(1),clus[i].state.at<float>(2)};

    }
}

Reflect AGNES::mergeProjection(std::vector<DataPoint> clus) {
    Reflect res;
    for (int i = 0; i < clus.size(); i++)
    {
//        sum_data1+=C1[i].state;


        res.point_num+=clus[i].reflect.point_num;
        Eigen::Vector3f  temp={clus[i].state.at<float>(0),clus[i].state.at<float>(1),clus[i].state.at<float>(2)};
    }
}

DataPoint AGNES::mergeData(std::vector<DataPoint> clus,int index) {
    DataPoint res;
    Eigen::Vector3f vel;
    for (auto clu:clus)
    {
        if(index>=0&&clu.clusterID!=index){
            res.enable= false;
            return res;
        }
//        if (index>=0)
//            std::cout<<clu.classID<<" ";
        res.pointCloud+=clu.pointCloud;
        res.reflect.rect|=clu.reflect.rect;
        res.reflect.vel+=clu.reflect.vel*clu.reflect.point_num;
    }

//    if (index>=0)
//        std::cout<<std::endl;
    res.reflect.point_num=res.pointCloud.size();
    res.reflect.vel/=res.reflect.point_num;
    res.state=cv::Mat::zeros(12,1,CV_32FC1);
    auto state_six= calculateBbox(res.pointCloud);
    for (int i = 0; i < state_six.size(); ++i) {
        res.state.at<float>(i)=state_six.at(i);
    }
    res.state.at<float>(6)=vel.x();
    res.state.at<float>(7)=vel.y();
    res.state.at<float>(8)=vel.z();
    res.ID=clus.at(0).ID;
//    res.classID=clus.at(0).classID;
    res.enable= true;
    return res;
}

//两个聚类之间最小的距离
double AGNES::getCDistance(std::vector<DataPoint> C, std::vector<DataPoint> C2)
{

    double MinDis = 9999;
    for (int i = 0; i < C.size(); i++)
    {

        for (int j = 0; j < C2.size(); j++)
        {

            double dis = getDistance(C[i], C2[j]);
            if (dis < MinDis)
            {
                MinDis = dis;
            }
        }
    }
    return MinDis;
}
//两个聚类的平均距离
double AGNES::getAVGDistance(std::vector<DataPoint> &C1, std::vector<DataPoint> &C2)
{
//    cv::Mat sum_data1(12,1,CV_32F,cv::Scalar(0)),sum_data2(12,1,CV_32F,cv::Scalar(0));
    Eigen::Vector3f v1,v2,p1,p2;
    Reflect reflect1,reflect2;
    auto merged1= mergeData(C1,-1);
    auto merged2= mergeData(C2,-1);
    reflect1=merged1.reflect;
    reflect2=merged2.reflect;
    p1={merged1.state.at<float>(0),merged1.state.at<float>(1),merged1.state.at<float>(2)};
    p2={merged2.state.at<float>(0),merged2.state.at<float>(1),merged2.state.at<float>(2)};
    reflect1.vel/=reflect1.point_num;
    v1=reflect1.vel;
    reflect2.vel/=reflect2.point_num;
    v2=reflect2.vel;
    float  dis_vel= v1.dot(v2)/(v1.norm()*v2.norm());
    float dis_pos= (p2-p1).norm();
    if(dis_pos>ThreshordDis*2)
        dis_vel=0;
    float  cost_max1=0,cost_max2=0,index1=-1,index2=-1;
    if(model==1){
        for (int i = 0; i < reflects2d.size(); ++i) {
            float cost1 = calculateIou(reflect1,reflects2d[i]);
            float cost2 = calculateIou(reflect2,reflects2d[i]);
            if(cost1>cost_max1) {
                cost_max1=cost1;
                index1=i;
            }
            if(cost2>cost_max2) {
                cost_max2=cost2;
                index2=i;
            }
        }
    }
//    if(index1>=0&&cost_max1>ThreshordDis){
////        for (auto &clu:C1) {
////            //赋值属性分类标签
////            clu.classID=reflects2d.at(index1).classID;
//////            clu.ID=reflects2d.at(index1).ID;
////        }
//    }
    if(model==0||index1==-1&&index2==-1){
        return dis_pos-dis_vel;
    }
    if(index1>=0&&index2>=0&&index1==index2){
        //属于同一类 正向增益
//        std::cout<<"seg buffer:"<<index1<<" "<<std::endl;
        return dis_pos-dis_vel-cost_max1-cost_max2;
    }else{
        //不属于同一类 反向增益
        return dis_pos-dis_vel+cost_max1+cost_max2;
    }
}
std::vector<DataPoint> AGNES::reportCluster() {
    std::vector<DataPoint> res;
    std::cout<<"2d:"<<reflects2d.size()<<std::endl;
    for (int  i=0;i<CluData.size();i++) {
        auto merged= mergeData(CluData[i],i);
        if (merged.enable== true){
            float cost_max=0;
            int index_obj;
            for (int i = 0; i < reflects2d.size(); ++i) {
                float cost= calculateIou(merged.reflect,reflects2d[i]);
                if(cost>cost_max) {
                    cost_max=cost;
                    index_obj=i;
                }
            }
            if(cost_max>ThreshordDis){
                std::cout<<index_obj<<" "<<cost_max<<":"<<std::endl;
                for (auto c:CluData[i]) {
                    std::cout<<c.clusterID<<" "<<std::endl;
                }
                merged.classID=reflects2d[index_obj].classID;
            }
            res.push_back(merged);
        }
    }
    return res;
}