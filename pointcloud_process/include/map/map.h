#ifndef _MAP_H
#define _MAP_H


#include <vector>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>   
#include "matrix.h"

namespace MapGet{

class MapConverter{
    public:
        
        //map值分为三个 255：障碍物 150：障碍物膨胀范围 0：无障碍物
        //在此基础上+0： 未搜索 +1： 目前在openlist +2: 目前在closelist
        void SetMapParam(const nav_msgs::OccupancyGrid& Map, matrix& gridmap, float plan_tolerance, float radius, int occupy_thresh, bool use_unknown);
        void MapToWorld(int mx, int my, float& wx, float& wy);
        bool WorldToMap(int& mx, int& my, float wx, float wy);
        void SetUnreachable(int mx, int my, matrix& gridmap);
        void SetUnreachableDy(int mx, int my, matrix& gridmap,float rad);
       bool CheckReachable(int mx, int my, matrix &gridmap,float thre);
       float GetResolution(){return resolution_;};
        float GetOriginx(){return origin_x_;};
        float GetOriginy(){return origin_y_;};
        

        float resolution_, plan_tolerance_, radius_;
        int height_, width_;
        float origin_x_, origin_y_;
        int occupy_thresh_;    
        bool use_unknown_; 
        bool initialized_ = false;    

    private:
        
};
    

        
}

#endif