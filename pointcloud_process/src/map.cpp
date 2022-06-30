#include "map/map.h"
#include "map/matrix.h"
#include <vector>

void MapGet::MapConverter::MapToWorld(int mx, int my, float& wx, float& wy)
{
    wx = mx*resolution_ + origin_x_;
    wy = my*resolution_ + origin_y_;
}

bool MapGet::MapConverter::WorldToMap(int& mx, int& my, float wx, float wy)
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;
    mx = round((wx-origin_x_)/resolution_);
    my = round((wy-origin_y_)/resolution_);
    if (mx < width_ && my < height_)
        return true;
    return false;
}

void MapGet::MapConverter::SetMapParam(const nav_msgs::OccupancyGrid& Map, matrix& gridmap, float plan_tolerance, float radius, int occupy_thresh=100, bool use_unknown=false)
{
    resolution_ = Map.info.resolution;
    plan_tolerance_ = plan_tolerance;
    radius_ = radius;
    width_ = Map.info.width;
    height_ = Map.info.height;
    origin_x_ = Map.info.origin.position.x;
    origin_y_ = Map.info.origin.position.y;
    occupy_thresh_ = occupy_thresh;
    use_unknown_ = use_unknown;
  //  if(!initialized_)
    {
         gridmap.create(width_, height_, 0);
        initialized_ = true;
     //   std::cout<<"map initialized!"<<std::endl;
    }
        
    for(int i=0;i<width_; i++)
        for(int j=0;j<height_;j++)
        {
            int map_value = Map.data[i+j*width_];
            // std::cout<<i<<","<<j<<map_value<<std::endl;
            if(map_value>=occupy_thresh_)
            {            
                SetUnreachable(i,j,gridmap);
            }
            else if(map_value<0 && !use_unknown_)
            {
                SetUnreachable(i,j,gridmap);
            }
            
        }
}
bool MapGet::MapConverter::CheckReachable(int mx, int my, matrix &gridmap,float thre) {
//    gridmap.SetElem(mx,my,255);
    int length = ceil((plan_tolerance_+2*radius_)/resolution_);
    int l = ceil(2*radius_/resolution_);
    int sum_ele=0;
    for(int i=-length;i<=length;i++)
        for(int j=-length;j<=length;j++)
        {
            if((mx+i)>=0 && (mx+i)<width_ && (my+j)>=0 && (my+j)<height_)
            {
                if(abs(i)<=l && abs(j)<=l)
                {
                    sum_ele+=gridmap.GetElem(mx+i, my+j);
                }
            }
        }
        int avg_ele=sum_ele/((2*length+1)*(2*length+1));
        if(avg_ele<thre) {
//            std::cout << "avg_ele:" << avg_ele << std::endl;
            return true;
        }
        return false;
}
void MapGet::MapConverter::SetUnreachable(int mx, int my, matrix& gridmap)
{
    gridmap.SetElem(mx,my,255);
    int length = ceil((plan_tolerance_+2*radius_)/resolution_);
    int l = ceil(2*radius_/resolution_);
    for(int i=-length;i<=length;i++)
        for(int j=-length;j<=length;j++)
        {
            if((mx+i)>=0 && (mx+i)<width_ && (my+j)>=0 && (my+j)<height_)
            {
                if(abs(i)<=l && abs(j)<=l)
                {
                    gridmap.SetElem(mx+i, my+j, 255);
                }
                if(gridmap.GetElem(mx+i, my+j)<255)
                    gridmap.SetElem(mx+i, my+j, 150);
            }
        }

    length = ceil(radius_/resolution_);
    for(int i=-length;i<=length;i++)
        for(int j=-length;j<=length;j++)
        {
            if((mx+i)>=0 && (mx+i)<width_ && (my+j)>=0 && (my+j)<height_)
            {
                if(gridmap.GetElem(mx+i, my+j)<255)
                    gridmap.SetElem(mx+i, my+j, 255);
            }
        }
}
void MapGet::MapConverter::SetUnreachableDy(int mx, int my, matrix& gridmap,float rad)
{
    gridmap.SetElem(mx,my,255);
    int length = ceil((plan_tolerance_+2*rad)/resolution_);
    int l = ceil(2*rad/resolution_);
    for(int i=-length;i<=length;i++)
        for(int j=-length;j<=length;j++)
        {
            if((mx+i)>=0 && (mx+i)<width_ && (my+j)>=0 && (my+j)<height_)
            {
                if(abs(i)<=l && abs(j)<=l)
                {
                    gridmap.SetElem(mx+i, my+j, 255);
                }
                if(gridmap.GetElem(mx+i, my+j)<255)
                    gridmap.SetElem(mx+i, my+j, 150);
            }
        }

    length = ceil(rad/resolution_);
    for(int i=-length;i<=length;i++)
        for(int j=-length;j<=length;j++)
        {
            if((mx+i)>=0 && (mx+i)<width_ && (my+j)>=0 && (my+j)<height_)
            {
                if(gridmap.GetElem(mx+i, my+j)<255)
                    gridmap.SetElem(mx+i, my+j, 255);
            }
        }
}