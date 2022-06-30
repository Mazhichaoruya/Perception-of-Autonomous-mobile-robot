#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h> // for PCL_ERROR
#include <pcl/pcl_base.h>
#include <pcl/search/search.h> // for Search
#include <pcl/search/kdtree.h> // for KdTree

template <typename PointT>
struct ClusterPre{
    PointT center;
    int classID;
    Eigen::Vector3f history_size;
    float radius;
};
template <typename PointT>
class Cluster: public pcl::PCLBase<PointT>{
        using BasePCLBase = pcl::PCLBase<PointT>;
public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using KdTree = pcl::search::Search<PointT>;
        using KdTreePtr = typename KdTree::Ptr;

        using PointIndicesPtr = pcl::PointIndices::Ptr;
        using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

        Cluster () : tree_ (),
                                        cluster_tolerance_ (0),
                                        min_pts_per_cluster_ (1),
                                        max_pts_per_cluster_ (std::numeric_limits<pcl::uint16_t>::max ())
        {};

        inline void initSupervisedInfo (int  temp)
        {

        }

        inline void setClusterParam (float tolerance,int min_cluster_size,int max_cluster_size,const KdTreePtr &tree)
        {
            cluster_tolerance_ = tolerance;
            min_pts_per_cluster_ = min_cluster_size;
            max_pts_per_cluster_ = max_cluster_size;
            tree_ = tree;
        }

        void extractCloud (std::vector<pcl::PointIndices> &clusters);

private:

        // Members derived from the base class
        using BasePCLBase::input_;
        using BasePCLBase::indices_;
        using BasePCLBase::initCompute;
        using BasePCLBase::deinitCompute;

        KdTreePtr tree_;
        float cluster_tolerance_;
        int min_pts_per_cluster_;
        int  max_pts_per_cluster_;
        bool surpervised_enable=false;
        bool init_per = false;
        std::vector<ClusterPre<PointT>> supervised_info;
        ClusterPre<PointT> cur_cluster;
        void extractClusters (const pcl::PointCloud<PointT> &cloud,
                                                              const std::vector<int> &indices,
                                                              std::vector<pcl::PointIndices> &clusters);
        PointT calBox(PointT p1,PointT p2){
            PointT p0;
            p0.x= std::abs(p1.x-p2.x);
            p0.y= std::abs(p1.y-p2.y);
            p0.z= std::abs(p1.z-p2.z);
            return p0;
        }
};

template <typename PointT> void
Cluster<PointT>::extractClusters (const pcl::PointCloud<PointT> &cloud,
                                  const std::vector<int> &indices,
                                  std::vector<pcl::PointIndices> &clusters)
{
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    if (tree_->getInputCloud ()->points.size () != cloud.points.size ())
    {
        PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree_->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    if (tree_->getIndices ()->size () != indices.size ())
    {
        PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%lu) than the input set (%lu)!\n", tree_->getIndices ()->size (), indices.size ());
        return;
    }
    // Check if the tree is sorted -- if it is we don't need to check the first element
    int nn_start_idx = tree_->getSortedResults () ? 1 : 0;

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices,seed_indices;
    std::vector<float> nn_distances,seed_distances;
    // Process all points in the indices vector
    for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
    {
        if (processed[indices[i]])
            continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        //从监督信息初始化起始点
        if(!supervised_info.empty()&&tree_->nearestKSearch(supervised_info.end()->center,1,seed_indices,seed_distances)>0&& *seed_distances.begin()<cluster_tolerance_){
            seed_queue.push_back(*seed_indices.begin());
            processed[*seed_indices.begin()] = true;
            cur_cluster = *supervised_info.begin();
            supervised_info.pop_back();
            surpervised_enable=true;
        }
        else{
            seed_queue.push_back (indices[i]);
            processed[indices[i]] = true;
            surpervised_enable= false;
        }
        while (sq_idx < static_cast<int> (seed_queue.size ()))
        {
            // Search for sq_idx
            int ret;
            if(surpervised_enable&&init_per== false){
                ret = tree_->radiusSearch (cloud.points[seed_queue[sq_idx]], cur_cluster.radius, nn_indices, nn_distances);
            }else{
                ret = tree_->radiusSearch (cloud.points[seed_queue[sq_idx]], cluster_tolerance_, nn_indices, nn_distances);
            }
            if( ret == -1)
            {
                PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from radiusSearch\n");
                exit(0);
            }
            if (!ret)
            {
                sq_idx++;
                continue;
            }
            init_per=true;

            for (size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
            {
                if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
                    continue;

                if(surpervised_enable){
                    PointT  box = calBox(cur_cluster.center,cloud.points[nn_indices[j]]);
                    if(box.x>cur_cluster.history_size.x()||box.x>cur_cluster.history_size.y()||box.x>cur_cluster.history_size.z())
                        continue;
                }

                seed_queue.push_back (nn_indices[j]);
                processed[nn_indices[j]] = true;
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_)
        {
            pcl::PointIndices r;
            r.indices.resize (seed_queue.size ());
            for (size_t j = 0; j < seed_queue.size (); ++j)
                // This is the only place where indices come into play
                r.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            //r.indices.assign(seed_queue.begin(), seed_queue.end());
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
            r.header = cloud.header;
            clusters.push_back (r);   // We could avoid a copy by working directly in the vector
        }
    }
}

template <typename PointT> void
Cluster<PointT>::extractCloud(std::vector<pcl::PointIndices> &clusters)
{
    if (!initCompute () ||
        (input_ != 0   && input_->points.empty ()) ||
        (indices_ != 0 && indices_->empty ()))
    {
        clusters.clear ();
        return;
    }

    // Initialize the spatial locator
    if (!tree_)
    {
        if (input_->isOrganized ())
            tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
        else
            tree_.reset (new pcl::search::KdTree<PointT> (false));
    }

    // Send the input dataset to the spatial locator
    tree_->setInputCloud (input_, indices_);
    extractClusters (*input_, *indices_,clusters);

    // Sort the clusters based on their size (largest one first)
    std::sort (clusters.rbegin (), clusters.rend (), pcl::comparePointClusters);

    deinitCompute ();
}