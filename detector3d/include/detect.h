//
// Created by mzc on 2020/12/3.
//
#include "include.h"
#include "detector3d/object.h"
#include "lidar_pointcloud.h"
#include "kmeans.h"
#ifndef SRC_DETECT_H
#define SRC_DETECT_H
using namespace std;
struct ObjectImgtoDepth {
    cv::Rect Rimg;
    int index;
    detector3d::object object;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Pointcloud;
    int resultID;
};
struct Objects{
    float IOU_Threshold_max,IOU_Threshold_min;
    vector<ObjectImgtoDepth> objects;
    vector<Cluster> clusters;
    vector<vector<float>> VecIOU;
    float CalIOU(cv::Rect Ri,cv::Rect Rl);
    bool BoxesMatch();
    void initobjects();
};
float Objects::CalIOU(cv::Rect Ri, cv::Rect Rl) {
    cv::Rect rect1 =    Ri  |   Rl;
    cv::Rect rect2 =    Ri  &   Rl;
    float IOU=rect2.area()*1.0/rect1.area();
    return IOU ;
}
bool Objects::BoxesMatch() {
//    cout<<"obj:"<<objects.size()<<" clu:"<<clusters.size()<<endl;
    for (auto obj:objects){
        vector<float> IOUObj,copyx,copyx_,areaRmix,areaplus;
        vector<cv::Rect> Rmix;
        for (int i = 0; i < clusters.size(); ++i) {
            Rmix.push_back(clusters.at(i).R & obj.Rimg);
            copyx.push_back(abs(clusters.at(i).center_point.x));
            areaRmix.push_back(Rmix.at(i).area());
            areaplus.push_back((obj.Rimg|clusters.at(i).R).area());
        }
        copyx_=copyx;
        sort(copyx_.begin(),copyx_.end());
        for (int i = 0; i < copyx.size(); ++i) {
            auto iter=find(copyx_.begin(),copyx_.end(),copyx.at(i));
            clusters.at(i).index=iter-copyx_.begin();
        }
        for (int i = 0; i < clusters.size(); ++i) {
            for (int j = 0; j < Rmix.size(); ++j) {
                if (clusters.at(i).index>clusters.at(j).index)
                    areaRmix.at(i)-=(Rmix.at(i)&Rmix.at(j)).area();
            }
            IOUObj.push_back(areaRmix.at(i)/areaplus.at(i));
        }
//        for (auto clu:clusters){
//            IOUObj.push_back(CalIOU(obj.Rimg,clu.R));
//        }
        VecIOU.push_back(IOUObj);
    }
//    cout<<"sizeIOU:"<<VecIOU.size()<<endl;
//    for (int i = 0; i < VecIOU.size(); ++i) {
//        for (int j = 0; j < VecIOU.at(i).size(); ++j) {
//            cout<<VecIOU.at(i).at(j)<<endl;
//        }
//    }
    for (int i = 0; i < VecIOU.end()-VecIOU.begin(); ++i) {
//        std::cout<<"i="<<i<<endl;
        vector<float> copy=VecIOU.at(i);
        sort(copy.begin(),copy.end());//按IOU大小排序
        if (copy.at(copy.size()-1)>IOU_Threshold_max){//IOU大于0.5则认为匹配
            auto iter=find(VecIOU.at(i).begin(),VecIOU.at(i).end(),copy.at(copy.size()-1));
            objects.at(i).index=iter-VecIOU.at(i).begin()+1;
//            cout<<"IOU="<<copy.at(copy.size()-1)<<endl;
        }
        if (copy.at(copy.size()-1)<IOU_Threshold_min){//IOU小于0.05则认为匹配失败
            objects.at(i).index=0;
        }
        if (copy.at(copy.size()-1)>IOU_Threshold_min&&copy.at(copy.size()-1)<IOU_Threshold_max){//IOU介于0.05-0.5之间则认为找到目标需要额外处理--目标位于某个聚类单元内部需要进一步提取
            auto iter=find(VecIOU.at(i).begin(),VecIOU.at(i).end(),copy.at(copy.size()-1));
//            if(objects.at(i).Rimg.area()>clusters.at(iter-VecIOU.at(i).begin()).R.area())
//                objects.at(i).index=(iter-VecIOU.at(i).begin()+1);
//            else
            objects.at(i).index=-(iter-VecIOU.at(i).begin()+1);
//            cout<<"IOU="<<copy.at(copy.size()-1)<<endl;
        }
    }
}
template <typename PointCloudPtrType>
void show_point_cloud(PointCloudPtrType cloud, std::string display_name) {
    pcl::visualization::CloudViewer viewer(display_name);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}
void Objects::initobjects() {
    if (LidarEnable){
        IOU_Threshold_max=0.5;IOU_Threshold_min=0.05;
    } else if(DepthEnable){
        IOU_Threshold_max=0.5;IOU_Threshold_min=0.1;
    }
    for (int i = 0; i <objects.size(); ++i) {
        objects.at(i).Pointcloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        if (objects.at(i).index>0){
          objects.at(i).Pointcloud=clusters.at(objects.at(i).index-1).pointcould;
            objects.at(i).resultID=1;
            cout<<"Matched successfully Only by First Clustered! "<<objects.at(i).object.classname<<" poincloud size:"<<objects.at(i).Pointcloud->size()<<endl;
        }
        else if (objects.at(i).index==0){//失败
            objects.at(i).Pointcloud->points.clear();
            objects.at(i).resultID=0;
            cout<<"Matched Failed! "<<objects.at(i).object.classname<<endl;
        }else{ ////进一步 半监督Kmeans聚类操作
            KMeans kmclusters;
            kmclusters.SetK(3);//3类中 0 1 2 依次为目标类 非目标类  其他
            kmclusters.rectobj=objects.at(i).Rimg;//将目标检测的矩形框载入
            kmclusters.rectclu=clusters.at(-objects.at(i).index-1).R;//当前聚类框
            kmclusters.SetInputCloud(clusters.at(-objects.at(i).index-1).pointcould);
            kmclusters.Cluster();
//            cout<<"poincloud size:"<<kmclusters.m_grp_pntcloud.at(0).size()<<" "<<kmclusters.m_grp_pntcloud.at(1).size()<<" "<<kmclusters.m_grp_pntcloud.at(2).size()<<endl;
            for(auto p:kmclusters.m_grp_pntcloud.at(0)){
                pcl::PointXYZI pI;
                pI.x=p.pnt.x;
                pI.y=p.pnt.y;
                pI.z=p.pnt.z;
                pI.intensity=i;
                objects.at(i).Pointcloud->push_back(pI);
            }
            //调试Kmeans聚类效果
//            pcl::PointCloud<pcl::PointXYZI>::Ptr pckmeans(new pcl::PointCloud<pcl::PointXYZI>);
//            for (auto pck_cloud:kmclusters.m_grp_pntcloud) {
//                for(auto p:pck_cloud){
//                    pcl::PointXYZI pI;
//                    pI.x=p.pnt.x;
//                    pI.y=p.pnt.y;
//                    pI.z=p.pnt.z;
//                    pI.intensity=p.groupID+1;
//                    pckmeans->points.push_back(pI);
//                }
//            }
//            show_point_cloud(pckmeans, "pointcloud pckmeans");
            cout<<"Matched by Twice Clustered successfully! "<<objects.at(i).object.classname<<" poincloud size:"<<kmclusters.m_grp_pntcloud.at(0).size()<<endl;
            objects.at(i).resultID=2;
        }
    }
}
#endif //SRC_DETECT_H
