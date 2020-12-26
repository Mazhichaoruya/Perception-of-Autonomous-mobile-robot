//
// Created by mzc on 2020/12/10.
//

#include "depth_pointcloud.h"

template <typename PointCloudPtrType>
void depth_pointcloud:: show_point_cloud(PointCloudPtrType cloud, std::string display_name) {
    pcl::visualization::CloudViewer viewer(display_name);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}
void depth_pointcloud::init(cv::Mat &depthmat,int id){
    cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_f.reset(new pcl::PointCloud<pcl::PointXYZI>);
    tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);
    cam.index=id;
    cam.imagewidth=depthmat.rows;
    cam.imageheigth=depthmat.cols;
    int Threshold;
    if (id==0){
        cam.Camerainfo=Inner_Transformation_Depth;
        cam.TLtoC=V_T;
        cam.RLtoC=MTR;
        camrgb.TLtoC=T_deptoimg;
        camrgb.RLtoC=R_deptoimg;
        camrgb.Camerainfo=InnerTransformation_Color;
        Threshold==3000;
    }
    if (id==1){
        cam.Camerainfo=Inner_Transformation_Depth1;
        cam.TLtoC=V_T1;
        cam.RLtoC=MTR1;
        camrgb.TLtoC=T_deptoimg1;
        camrgb.RLtoC=R_deptoimg1;
        camrgb.Camerainfo=InnerTransformation_Color1;
        Threshold=5000;
    }
    for (int i = 0; i < depthmat.rows; ++i) {
        for (int j = 0; j < depthmat.cols; ++j) {
            if (depthmat.at<uint16_t>(i, j) > 0 && depthmat.at<uint16_t>(i, j) < Threshold)//D435取测量范围为0-4m
            {
            Eigen::Vector3f point3d =depthmat.at<uint16_t>(i, j)/1000.0*cam.RLtoC.inverse()*(cam.Camerainfo.inverse()*Eigen::Vector3f{j, i, 1}-cam.TLtoC);//转为深度相机坐标系 *MTR.inverse()
            pcl::PointXYZI point;
            point.x=point3d.x();point.y=point3d.y();point.z=point3d.z();
            if (point.z>-0.27)//机器人设置高度-0.27
                cloud->push_back(point);
            }
        }
    }
    cloudsize=cloud->size();
    cluster_indices.clear();
}
void depth_pointcloud::pointcloud_predeal() {
    // Filtering [Downsampling a PointCloud using a VoxelGrid filter](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search)
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.filter(*cloud_f);
    int downsampled_points_total_size = cloud_f->points.size();
    std::cout << "PointCloud after filtering has: " << downsampled_points_total_size  << " data points." << std::endl;
//    show_point_cloud(cloud_f, "downsampled point cloud");
/*弃用RANSAC 寻找最大平面不稳定
    // remove the biggest plane
    // Segmentation, Ransac, [Plane model segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation)
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar anymore." << std::endl;
    } else {
        // Filter, [Extracting indices from a PointCloud](https://pcl.readthedocs.io/projects/tutorials/en/latest/extract_indices.html#extract-indices)
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        pcl::PointXYZI min, max;
        pcl::getMinMax3D(*cloud_filtered, min, max);
//        double min_z = min.z;
//        std::cout << "ground plane size: " << cloud_plane->points.size()  << ", min_z:" << min_z << std::endl;
//        show_point_cloud(cloud_plane, "gound plane in point cloud");
        // filter planar
        extract.setNegative(true);
        extract.filter(*cloud_f);
//        show_point_cloud(cloud_f, "plane filtered point cloud");
        *cloud_filtered = *cloud_f;
    }
*/

}
void depth_pointcloud::pointcloud_cluster(){
//    show_point_cloud(cloud, "original pointcloud");
    clock_t start_ms = clock();
    pointcloud_predeal();
    vec_clu.clear();
    int j = 0,index=0;
    Eigen::Vector3f point3d;
    Cluster Clu_tmp;
    // KdTree, for more information, please ref [How to use a KdTree to search](https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search)
    tree->setInputCloud(cloud_f);
    // test 1. uncomment the following two lines to test the simple dbscan
    // DBSCANSimpleCluster<pcl::PointXYZI> ec;
    // ec.setCorePointMinPts(20);

    // test 2. uncomment the following two lines to test the precomputed dbscan
    // DBSCANPrecompCluster<pcl::PointXYZI>  ec;
    // ec.setCorePointMinPts(20);

    // test 3. uncomment the following two lines to test the dbscan with Kdtree for accelerating
    ec.setCorePointMinPts(20);

    // test 4. uncomment the following line to test the EuclideanClusterExtraction

    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_f);
    ec.extract(cluster_indices);
    std::cout<<cluster_indices.end()-cluster_indices.begin()<<std::endl;
    //visualization, use indensity to show different color for each cluster.
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, j++) {
        int minx = cam.imagewidth, miny = cam.imageheigth, maxx = 0, maxy = 0;
        Clu_tmp.pointcould.reset(new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            pcl::PointXYZI tmp;
            //雷达坐标系到相机坐标系坐标轴变化
            point3d.x() = tmp.x = cloud_f->points[*pit].x;

            point3d.y() = tmp.y = cloud_f->points[*pit].y;

            point3d.z() = tmp.z = cloud_f->points[*pit].z;
            tmp.intensity = j;
            cloud_cluster->points.push_back(tmp);
            auto B = camrgb.Camerainfo*(camrgb.RLtoC * (cam.RLtoC*point3d+cam.TLtoC) + camrgb.TLtoC);
            Eigen::Vector3f imgpix = B / B.z();
//            std::cout<<"B:"<<B.x()<<" "<<B.y()<<" "<<B.z()<<std::endl;
//            std::cout<<"imgpix:"<<imgpix.x()<<" "<<imgpix.y()<<" "<<imgpix.z()<<std::endl;
            if (imgpix.x() >= 0 && imgpix.y() >= 0 && B.z() > 0 && imgpix.x() < cam.imagewidth - 1 &&imgpix.y() < cam.imageheigth - 1) {
                if (minx > imgpix.x())
                    minx = imgpix.x();
                if (miny > imgpix.y())
                    miny = imgpix.y();
                if (maxx < imgpix.x())
                    maxx = imgpix.x();
                if (maxy < imgpix.y())
                    maxy = imgpix.y();
                Clu_tmp.pointcould->push_back(tmp);
            }
        }
            if (!Clu_tmp.pointcould->empty()) {
                Clu_tmp.index = index;
                Clu_tmp.cloudsize = Clu_tmp.pointcould->size();
                Clu_tmp.R = cv::Rect(minx, miny, (maxx - minx), (maxy - miny));
                vec_clu.push_back(Clu_tmp);
//            std::cout<<"RECT:"<<index<<":<"<<minx<<","<<miny<<"><"<<maxx<<","<<maxy<<"> size:"<<Clu_tmp.cloudsize<<std::endl;
                index++;
            }
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
//    show_point_cloud(cloud_cluster, "colored clusters of point cloud");//可视化
    clock_t end_ms = clock();
//    std::cout << "cluster time cost:" << double(end_ms - start_ms) / CLOCKS_PER_SEC << " s" << std::endl;
}
