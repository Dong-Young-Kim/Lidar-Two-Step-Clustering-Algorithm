#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions (struct termios)
#include <system_error>	// For throwing std::system_error
#include <math.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <two_step_clustering_algorithm/FastClustering_dy.h>

namespace TSC {
    class TwoStepClustering{
    public:
        TwoStepClustering();
        void Run();
        void FirstClustering();
        void SecondClustering();
        void AfterClustering();
        
        void Publish();
        void InitNode();

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;

        pcl::PointCloud<pcl::PointXYZI> inputCloud;
        void SubscribeCallback(const sensor_msgs::PointCloud2ConstPtr& input_data);

    };
}

TSC::TwoStepClustering TwoStepClustering(){
    this->InitNode();
}

void TSC::TwoStepClustering::InitNode(){
    pub = nh.advertise<sensor_msgs::PointCloud2> ("two_step_clustering", 1);
    sub = nh.subscribe<sensor_msgs::PointCloud2> ("/2_1_velodyne_points_ransac", 1, SubscribeCallback, this);
}

void TSC::TwoStepClustering::Run(){
    while(ros::ok()){
        ros::spinOnce();
    }
}

void TSC::TwoStepClustering::SubscribeCallback(const sensor_msgs::PointCloud2ConstPtr& input_data){
    pcl::fromROSMsg(*input_data, inputCloud);
}