//
// Created by mzc on 2020/12/7.
//
#include "kmeans.h"
using namespace std;
const float DIST_NEAR_ZERO = 0.001;
bool KMeans::InitKCenter( )//m_k ==3
{
    mv_center.resize(m_k);
    int size = mv_pntcloud.size();
   /*
   ////随机初始化
    srand(unsigned(time(NULL)));
    for (int i =0; i< m_k;i++)
    {
        int seed = random()%(size+1);
        mv_center[i].x = mv_pntcloud[seed].pnt.x;
        mv_center[i].y = mv_pntcloud[seed].pnt.y;
        mv_center[i].z = mv_pntcloud[seed].pnt.z;
    }
    */
    Eigen::Vector3f point3d;
    st_pointxyz zero;
    zero.x=0;zero.y=0;zero.z=0;
    float last_in=MAXFLOAT,last_out_max=0.0,last_in_max=0.0,centerx,centery;
    centerx=rectobj.x+rectobj.width/2;centery=rectobj.y+rectobj.height/2;
    float r1=(abs(rectclu.y-rectobj.y)+rectclu.height/2)*(abs(rectclu.y-rectobj.y)+rectclu.height/2)+(abs(rectclu.x-rectobj.x)+rectclu.width/2)*(abs(rectclu.x-rectobj.x)+rectclu.width/2);
    float r2=(rectobj.height/2)*(rectobj.height/2)+(rectobj.width/2)*(rectobj.width/2);
    for(int i=0;i<mv_pntcloud.size();++i) {
        point3d.x() = mv_pntcloud.at(i).pnt.x;
        point3d.y() = mv_pntcloud.at(i).pnt.y;
        point3d.z() = mv_pntcloud.at(i).pnt.z;
        auto B = InnerTransformation_Color * MTR * (point3d + V_T);
        Eigen::Vector3f imgpix = B / B.z();
        auto distance=DistBetweenPoints(mv_pntcloud.at(i).pnt,zero);
        if (imgpix.x()>=rectobj.x&&imgpix.y()>=rectobj.y&&imgpix.x()<rectobj.x+rectobj.width&&imgpix.y()<rectobj.y+rectobj.height){//落在目标检测框内的点云
         mv_pntcloud.at(i).connection_constraint={-2,2,0};  //目标 框内增加必连约束 0为目标类 1为非目标点 2为其他类
        if(distance<last_in){//目标类起始点是目标框内最近点
            mv_center.at(0)=mv_pntcloud.at(i).pnt;
            last_in=distance;
            }
            if(distance>last_in_max){//其他点选择目标区域内最远的点
                mv_center.at(2)=mv_pntcloud.at(i).pnt;
                last_in_max=distance;
            }
        mv_pntcloud.at(i).weight=1-((imgpix.x()-centerx)*(imgpix.x()-centerx)+(imgpix.y()-centery)*(imgpix.y()-centery))/r2;//在目标区域内的约束权重以单位圆均匀分布
        }
        else{
            mv_pntcloud.at(i).connection_constraint={2,-2,0};//目标 框外增加勿连约束
            if(distance>last_out_max){ //非目标点--框外最远点作为起始点
                mv_center.at(1)=mv_pntcloud.at(i).pnt;
                last_out_max=distance;
            }
            mv_pntcloud.at(i).weight=((imgpix.x()-centerx)*(imgpix.x()-centerx)+(imgpix.y()-centery)*(imgpix.y()-centery))/r1;//在目标区域外的约束权重以单位圆均匀分布
        }
    }
    return true;
}
bool KMeans::SetInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pPntCloud)
{
    size_t pntCount = (size_t) pPntCloud->points.size();
    for (size_t i = 0; i< pntCount;++i)
    {
        st_point point;
        point.pnt.x = pPntCloud->points[i].x;
        point.pnt.y = pPntCloud->points[i].y;
        point.pnt.z = pPntCloud->points[i].z;
        point.groupID = 0;

        mv_pntcloud.push_back(point);
    }

    return true;
}
bool KMeans::Cluster()
{
    InitKCenter();
    vector<st_pointxyz>v_center(mv_center.size());
    size_t pntCount = mv_pntcloud.size();

    do
    {
        for (size_t i = 0;i < pntCount;++i)
        {
            double min_dist = DBL_MAX;
            int pnt_grp = 0;   //聚类群组索引号
            for (size_t j =0;j <m_k;++j)
            {
//                double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j]);//
                double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j])+mv_pntcloud.at(i).connection_constraint[j]*mv_pntcloud.at(i).weight;//增加成对约束权重因素
                if (min_dist - dist > 0.000001)
                {
                    min_dist = dist;
                    pnt_grp = j;
                }
            }
            m_grp_pntcloud[pnt_grp].push_back(st_point(mv_pntcloud[i].pnt,pnt_grp)); //将该点和该点群组的索引存入聚类中
        }

        //保存上一次迭代的中心点
        for (size_t i = 0; i<mv_center.size();++i)
        {
            v_center[i] = mv_center[i];
        }

        mv_center=UpdateGroupCenter(m_grp_pntcloud,mv_center);
        if ( !ExistCenterShift(v_center, mv_center))
        {
            break;
        }
        for (size_t i = 0; i < m_k; ++i){
            m_grp_pntcloud[i].clear();
        }

    }while(true);

    return true;
}
double KMeans::DistBetweenPoints(st_pointxyz &p1, st_pointxyz &p2)
{
    double dist = 0;
    double x_diff = 0, y_diff = 0, z_diff = 0;

    x_diff = p1.x - p2.x;
    y_diff = p1.y - p2.y;
    z_diff = p1.z - p2.z;
//    dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    dist = x_diff * x_diff + y_diff * y_diff + z_diff * z_diff;//不必使用开方运算增加运算量

    return dist;
}
vector<st_pointxyz> KMeans::UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyz> center)
{
    for (size_t i = 0; i < m_k; ++i)
    {
        float x = 0, y = 0, z = 0;
        size_t pnt_num_in_grp = grp_pntcloud[i].size();

        for (size_t j = 0; j < pnt_num_in_grp; ++j)
        {
            x += grp_pntcloud[i][j].pnt.x;
            y += grp_pntcloud[i][j].pnt.y;
            z += grp_pntcloud[i][j].pnt.z;
        }
        x /= pnt_num_in_grp;
        y /= pnt_num_in_grp;
        z /= pnt_num_in_grp;
        center[i].x = x;
        center[i].y = y;
        center[i].z = z;
    }
    return center;

}
//是否存在中心点移动
bool KMeans::ExistCenterShift(std::vector<st_pointxyz> &prev_center, std::vector<st_pointxyz> &cur_center)
{
    for (size_t i = 0; i < m_k; ++i)
    {
        double dist = DistBetweenPoints(prev_center[i], cur_center[i]);
        if (dist > DIST_NEAR_ZERO)
        {
            return true;
        }
    }

    return false;
}
//将聚类的点分别存到各自的pcd文件中
/*bool KMeans::SaveFile(const char *prex_name)
{
    for (int i = 0; i < m_k; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

        for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)
        {
            pcl::PointXYZ pt;
            pt.x = m_grp_pntcloud[i][j].pnt.x;
            pt.y = m_grp_pntcloud[i][j].pnt.y;
            pt.z = m_grp_pntcloud[i][j].pnt.z;

            p_pnt_cloud->points.push_back(pt);
        }

        p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();
        p_pnt_cloud->height = 1;

        char newFileName[256] = {0};
        char indexStr[16] = {0};

        strcat(newFileName, szFileName);
        strcat(newFileName, "-");
        strcat(newFileName, prex_name);
        strcat(newFileName, "-");
        sprintf(indexStr, "%d", i + 1);
        strcat(newFileName, indexStr);
        strcat(newFileName, ".pcd");
        pcl::io::savePCDFileASCII(newFileName, *p_pnt_cloud);
    }

    return true;
}*/

