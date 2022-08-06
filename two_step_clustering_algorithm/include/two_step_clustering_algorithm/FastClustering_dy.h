# pragma once

#include <boost/format.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>

template <typename PointT>
class FastClustering{
public:
    void setInputCloud              (typename pcl::PointCloud<PointT>::Ptr cloud)        {input_cloud = cloud;}
    void setSearchMethod            (typename pcl::search::KdTree<PointT>::Ptr tree)     {search_method = tree;}    
    void setClusterTolerance        (double tolerance)                          {cluster_tolerance = tolerance;}
    void setClusterToleranceAngle   (double tolerance_angle)                    {cluster_tolerance_angle = tolerance_angle;}
    void setMinClusterSize          (int min_cluster_size)                      {min_pts_per_cluster = min_cluster_size;}
    void setMaxClusterSize          (int max_cluster_size)                      {max_pts_per_cluster = max_cluster_size;}
    void extract                    (std::vector<pcl::PointIndices>& cluster_indices);
    
private:
    typename pcl::search::KdTree<PointT>::Ptr search_method;
    typename pcl::PointCloud<PointT>::Ptr input_cloud;
    double cluster_tolerance                = 0;
    double cluster_tolerance_angle          = 0;
    int min_pts_per_cluster                 = 1;
    int max_pts_per_cluster                 = std::numeric_limits<int>::max();
};

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
inline bool calculateTolerence (PointT point, double degree) {
    float distance = point.x * point.x + point.y * point.y + point.z * point.z;
    float sinAngle = (float)sin(degree * 3.14159265358979 / 180);
    return distance * sinAngle;
}

template <typename PointT>
void FastClustering<PointT>::extract(std::vector<pcl::PointIndices>& cluster_indices){

    if (search_method->getInputCloud ()->points.size () != input_cloud.points.size ()) {
        PCL_ERROR ("[extractFastClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", 
                    search_method->getInputCloud ()->points.size (), input_cloud->points.size ());
        return;
    }

    int nn_start_idx = search_method->getSortedResults () ? 1 : 0;

    std::vector<bool> processed (input_cloud->points.size(), false);
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;

    for(int i = 0; i < static_cast<int>(input_cloud->points.size()); ++i){
        if (processed[i]) continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back(i);
        processed[i] = true;

        while (sq_idx < static_cast<int>(seed_queue.size())){
            if (!search_method->radiusSearch (seed_queue[sq_idx], calculateTolerence(input_cloud->points[seed_queue[sq_idx]], cluster_tolerance_angle), nn_indices, nn_distances)){
                sq_idx++;
                continue;
            }
            for (int j = nn_start_idx; j < nn_indices.size (); ++j){
                if (nn_indices[j] == -1 || processed[nn_indices[j]]) continue;
                seed_queue.push_back (nn_indices[j]);
                processed[nn_indices[j]] = true;
            }
            sq_idx++;
        }

        if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster){
            pcl::PointIndices r;
            r.indices.resize (seed_queue.size ());
            for (int j = 0; j < seed_queue.size (); ++j) r.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = input_cloud->header;
            cluster_indices.push_back (r);
        }
    }
    std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
}