//
// Created by mzc on 2020/12/7.
//

#ifndef SRC_KMEANS_H
#define SRC_KMEANS_H
#include "include.h"
typedef struct st_pointxyz
{
    float x;
    float y;
    float z;
}st_pointxyz;

typedef struct st_point
{
    st_pointxyz pnt;
    int groupID;//增加初始化标签约束
    float weight;//必连|勿连约束惩罚权重
    std::vector<int> connection_constraint;//连接约束
    st_point()
    {
    }
    st_point(st_pointxyz &p,int id)
    {
        pnt =p;
        groupID= id;
    }
}st_point;

class KMeans
{
public:
    int m_k;
    typedef std::vector<st_point> VecPoint_t;  //定义命令别名
    cv::Rect rectobj,rectclu;
    VecPoint_t mv_pntcloud; //要聚类的点云
    std::vector<VecPoint_t>m_grp_pntcloud;  //k类，每一类存储若干点
    std::vector<st_pointxyz>mv_center; //每个类的中心

    KMeans()
    {
        m_k =0;
    }
    inline void SetK(int k_) //设置聚类簇数
    {
        m_k = k_;
        m_grp_pntcloud.resize(m_k);
    }
    //设置输入点云
    bool SetInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pPntCloud);

    //初始化最初的k个类的中心
    bool InitKCenter();

    //聚类
    bool Cluster();

    //更新k类的中心(参数为类和中心点)
    std::vector<st_pointxyz>  UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud,std::vector<st_pointxyz> cer);

    //计算两点欧式距离
    double DistBetweenPoints(st_pointxyz &p1,st_pointxyz &p2);

    //是否存在中心点转移动
    bool ExistCenterShift(std::vector<st_pointxyz> &prev_center,std::vector<st_pointxyz> &cur_center);

    //将聚类分别存储到各自的pcd文件中
    bool SaveFile(const char *fname);

};





#endif //SRC_KMEANS_H
