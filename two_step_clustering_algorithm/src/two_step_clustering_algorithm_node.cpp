#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <two_step_clustering_algorithm/FastClustering_dy.h>
#include <two_step_clustering_algorithm/cluster_process.h>


int main(int argc, char**argv) {
    ros::init(argc, argv, "two_step_clustering_algorithm");

    cout << "hello\n";

    TSC::TwoStepClustering TSClustering;
    TSClustering.Run();

}