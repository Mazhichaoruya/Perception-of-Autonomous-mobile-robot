//
// Created by mzc on 2020/11/28.
//

#include "lidar_pointcloud.h"
#include "cv_bridge/cv_bridge.h"
#include "include.h"
lidar_pointcloud::lidar_pointcloud() {
}
void lidar_pointcloud::initcamera(Eigen::Vector3f T,Eigen::Matrix<float,3,3> R,Eigen::Matrix<float,3,3> inner,int H,int W,int id){
    Camera_info camtemp;
    camtemp.TLtoC=T;
    camtemp.RLtoC=R;
    camtemp.Camerainfo=inner;
    camtemp.imagewidth=W;
    camtemp.imageheigth=H;
    camtemp.index=id;
    camtemp.max_y=0;
    camtemp.min_y=camtemp.imageheigth-1;
    cam_vec.at(id)=camtemp;
}
void lidar_pointcloud::init(pcl::PointCloud<pcl::PointXYZI>::Ptr pointclouds){
    cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloudIncamera.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
//    tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);
//    show_point_cloud(pointclouds, "original pointcloud");
    if (!SLAMEnable)
        for (auto point:pointclouds->points) {
            if (point.z>-0.35)
                cloud->push_back(point);
        }
    else
       cloud=pointclouds;
    cloudsize=cloud->size();
    cluster_indices.clear();
}
template <typename PointCloudPtrType>
void lidar_pointcloud:: show_point_cloud(PointCloudPtrType cloud, std::string display_name) {
    pcl::visualization::CloudViewer viewer(display_name);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}
void lidar_pointcloud::pointcloud_cluster(){
//    show_point_cloud(cloud, "pointcloud without ground");
//    lidar_clus=cv::Mat {imageheigth,imagewidth,CV_8UC1};
//    lidar_depth=cv::Mat {imageheigth,imagewidth,CV_16UC1};
    // Filtering [Downsampling a PointCloud using a VoxelGrid filter](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search)
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.filter(*cloud);
    int downsampled_points_total_size = cloud->points.size();
//    std::cout << "PointCloud after filtering has: " << downsampled_points_total_size  << " data points." << std::endl;
//    show_point_cloud(cloud, "downsampled point cloud");
    clock_t start_ms = clock();
//    lidar_depth=cv::Mat::zeros(cam_vec.at(0).imageheigth,cam_vec.at(0).imagewidth,CV_16UC1);
//    cv::Mat img(480,640,CV_8UC3,cv::Scalar(255, 255, 255));
//    lidar_clus=img;
    int j = 0;
    Eigen::Vector3f point3d;
    for(auto point:cloud->points){
        point3d.x() = point.x;
        point3d.y() = point.y;
        point3d.z() = point.z;
        for (auto &cam:cam_vec) {
            auto B=cam.Camerainfo*(cam.RLtoC*point3d+cam.TLtoC);
            Eigen::Vector3f imgpix=B/B.z();
//            Eigen::Vector3f imgpix=Camerainfo*RLtoC*(point3d-TLtoC)/tmp.z;
            if (imgpix.x()>=0&&imgpix.y()>=0&&B.z()>0&&imgpix.x()<cam.imagewidth-1&&imgpix.y()<cam.imageheigth-1) {
                cloudIncamera->push_back(point);
                if (imgpix.y()<cam.min_y)
                    cam.min_y=imgpix.y();
                if (imgpix.y()>cam.max_y)
                    cam.max_y=imgpix.y();
            }
        }
    }
//    for (auto cam:cam_vec){
//        cout<<"max_miny:"<<cam.max_y<<" "<<cam.min_y<<endl;
//    }
//    show_point_cloud(cloudIncamera, "Camera pointcloud");
    // KdTree, for more information, please ref [How to use a KdTree to search](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search)
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloudIncamera);
    // test 1. uncomment the following two lines to test the simple dbscan
    // DBSCANSimpleCluster<pcl::PointXYZI> ec;
    // ec.setCorePointMinPts(20);

    // test 2. uncomment the following two lines to test the precomputed dbscan
    // DBSCANPrecompCluster<pcl::PointXYZI>  ec;
    // ec.setCorePointMinPts(20);

    // test 3. uncomment the following two lines to test the dbscan with Kdtree for accelerating
//    DBSCANKdtreeCluster<pcl::PointXYZI> ec;
    // ec.setCorePointMinPts(20);

    // test 4. uncomment the following line to test the EuclideanClusterExtraction
    ec.setClusterTolerance(0.25);
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloudIncamera);
    ec.extract(cluster_indices);
//    std::cout<<cluster_indices.end()-cluster_indices.begin()<<std::endl;
    //visualization, use indensity to show different color for each cluster.
    vec_clu.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, j++) {
        for(auto cam:cam_vec){
            Cluster Clu_tmp;
            int minx=cam.imagewidth,miny=cam.imageheigth,maxx=0,maxy=0;
            float sumx=0.0,sumy=0.0,sumz=0.0;
            Clu_tmp.pointcould.reset(new pcl::PointCloud<pcl::PointXYZI>);
            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                pcl::PointXYZI tmp;
                //雷达坐标系到相机坐标系坐标轴变化
                point3d.x()=tmp.x = cloudIncamera->points[*pit].x;//bug crash位置**
                sumx+=point3d.x();
                point3d.y()=tmp.y = cloudIncamera->points[*pit].y;
                sumy+=point3d.y();
                point3d.z()=tmp.z = cloudIncamera->points[*pit].z;
                sumz+=point3d.z();
                tmp.intensity = j;
                cloud_cluster->points.push_back(tmp);
                auto B=cam.Camerainfo*cam.RLtoC*(point3d+cam.TLtoC);
                Eigen::Vector3f imgpix=B/B.z();
//            Eigen::Vector3f imgpix=Camerainfo*RLtoC*(point3d-TLtoC)/tmp.z;
            if (imgpix.x()>=0&&imgpix.y()>=0&&B.z()>0&&imgpix.x()<cam.imagewidth-1&&imgpix.y()<cam.imageheigth-1) {
//                std::cout<<"imgpix:"<<imgpix.x()<<" "<<imgpix.y()<<" "<<imgpix.z()<<std::endl;
                if (minx>imgpix.x())
                    minx=imgpix.x();
                if (miny>imgpix.y())
                    miny=imgpix.y();
                if (maxx<imgpix.x())
                    maxx=imgpix.x();
                if (maxy<imgpix.y())
                    maxy=imgpix.y();
                Clu_tmp.pointcould->push_back(tmp);
//                j=j%12;//最大j=11
//                lidar_clus.at<cv::Vec3b>(static_cast<int>(imgpix.y()), static_cast<int>(imgpix.x()))[0] = 125*j/4;//-j*255/(cluster_indices.end()-cluster_indices.begin());
//                lidar_clus.at<cv::Vec3b>(static_cast<int>(imgpix.y()), static_cast<int>(imgpix.x()))[1] =125*j/2;//-j*255/(cluster_indices.end()-cluster_indices.begin());
//                lidar_clus.at<cv::Vec3b>(static_cast<int>(imgpix.y()), static_cast<int>(imgpix.x()))[2] =125*(j%2+1);//-j*255/(cluster_indices.end()-cluster_indices.begin());
//                lidar_depth.at<uint16_t>(static_cast<int>(imgpix.y()), static_cast<int>(imgpix.x())) = static_cast<uint16_t>(imgpix.z());
            }
            }
            if (!Clu_tmp.pointcould->empty()){
                Clu_tmp.index=cam.index;//标记相机编号
                Clu_tmp.cloudsize=Clu_tmp.pointcould->size();
                Clu_tmp.R=cv::Rect(minx,miny,(maxx-minx),(maxy-miny));
                Clu_tmp.center_point.x=sumx/Clu_tmp.cloudsize;Clu_tmp.center_point.y=sumy/Clu_tmp.cloudsize;Clu_tmp.center_point.z=sumz/Clu_tmp.cloudsize;
//                cv::rectangle(lidar_clus, Clu_tmp.R, cv::Scalar(0, 0, 255), 1);
//                cv::rectangle(color_mat, Clu_tmp.R, cv::Scalar(0, 0, 255), 1);
                vec_clu.push_back(Clu_tmp);
//                cout<<"camera id:"<<Clu_tmp.index<<endl;
//            std::cout<<"RECT:"<<index<<":<"<<minx<<","<<miny<<"><"<<maxx<<","<<maxy<<"> size:"<<Clu_tmp.cloudsize<<std::endl;
            }
        }
    }
//    cout<<"all camera name:"<<vec_clu.size()<<endl;
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
//    show_point_cloud(cloud_cluster, "colored clusters of point cloud");//可视化
//    cv::imshow("boxshow",color_mat);
//    cv::waitKey(1);
    clock_t end_ms = clock();
    std::cout << "cluster time cost:" << double(end_ms - start_ms) / CLOCKS_PER_SEC << " s" << std::endl;

}
