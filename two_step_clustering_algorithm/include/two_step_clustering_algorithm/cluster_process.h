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
struct objInfo {
    pcl::PointIndices* objPoints;
    std::string classes;
    unsigned int idx;
    float x;
    float y;
    float z;
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;
    short intensity;
}; //순서 유지 필수

namespace TSC {
    class TwoStepClustering{
    public:
        TwoStepClustering();
        void Run();

    private:
        ros::NodeHandle     nh;
        ros::Publisher      pub_first;
        ros::Publisher      pub_second;
        ros::Subscriber     sub;

        pcl::PointCloud<pcl::PointXYZI>     inputCloud;
        pcl::PointCloud<pcl::PointXYZI>     firstClustered;
        pcl::PointCloud<pcl::PointXYZI>     seconedClustered;
        std::vector<pcl::PointIndices>      first_cluster_indices;
        std::vector<pcl::PointIndices>      second_cluster_indices;
        std::vector<struct objInfo>         objs;
    
        void InitNode();
        void SubscribeCallback(const sensor_msgs::PointCloud2ConstPtr& input_data);

        void AllClear();
        void FirstClustering();
        void SecondClustering();
        void AfterFirstClustering();
        sensor_msgs::PointCloud2 Publish(pcl::PointCloud<pcl::PointXYZI> pubCloud);

    };
}

TSC::TwoStepClustering::TwoStepClustering(){
    InitNode();
}

void TSC::TwoStepClustering::InitNode(){
    pub_first = nh.advertise<sensor_msgs::PointCloud2> ("two_step_clustering_first_clusterd", 1);
    sub = nh.subscribe<sensor_msgs::PointCloud2> ("/2_1_velodyne_points_ransac", 1, &TSC::TwoStepClustering::SubscribeCallback, this);
    
}

void TSC::TwoStepClustering::AllClear(){
    firstClustered.clear();
    seconedClustered.clear();
    first_cluster_indices.clear();
    second_cluster_indices.clear();
    objs.clear();
}

void TSC::TwoStepClustering::SubscribeCallback(const sensor_msgs::PointCloud2ConstPtr& input_data){
    cout << "data input\n";
    AllClear();
    pcl::fromROSMsg(*input_data, inputCloud);
    FirstClustering();
    AfterFirstClustering();
    //SecondClustering();

}

void TSC::TwoStepClustering::Run(){
    while(ros::ok()){
        ros::spinOnce();
    }
}

void TSC::TwoStepClustering::FirstClustering(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    *cloud = this->inputCloud;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    FastClustering<pcl::PointXYZI> fc;
    fc.setInputCloud(cloud);
    fc.setSearchMethod(tree);
    fc.setMinClusterSize(1);
    fc.setMaxClusterSize(1000);
    fc.setClusterToleranceAngle(0.5); //0.01 for safe cluster
    fc.extract(this->first_cluster_indices);
}

void TSC::TwoStepClustering::SecondClustering(){

}

void TSC::TwoStepClustering::AfterFirstClustering(){
    int intensityValue = 0;
    for (std::vector<pcl::PointIndices>::iterator it = this->first_cluster_indices.begin(); it != this->first_cluster_indices.end(); ++it, intensityValue++){

        std::pair<float,float> x(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); //first = min, second = max
        std::pair<float,float> y(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
        std::pair<float,float> z(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());

    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            pcl::PointXYZI pt = inputCloud.points[*pit];
            pt.intensity = intensityValue % 10;
            this->firstClustered.push_back(pt);
            if(pt.x < x.first)      x.first = pt.x;
            if(pt.x > x.second)     x.second = pt.x;
            if(pt.y < y.first)      y.first = pt.y;
            if(pt.y > y.second)     y.second = pt.y;
            if(pt.z < z.first)      z.first = pt.z;
            if(pt.z > z.second)     z.second = pt.z;
    	}

        objInfo tmp_obj = {&(*it), "unknown", (unsigned int)intensityValue, 
                            (x.first+x.second)/2, (y.first+y.second)/2, (z.first+z.second)/2,
                            x.first, y.first, z.first, x.second, y.second, z.second,
                            (short)(intensityValue % 10)};
        objs.push_back(tmp_obj);
    }
    cout << "firstClustered size  = " << firstClustered.size() << endl;
    pub_first.publish(this->Publish(firstClustered));

}

sensor_msgs::PointCloud2 TSC::TwoStepClustering::Publish(pcl::PointCloud<pcl::PointXYZI> pubCloud){
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 tmp_PCL;                                    //declare PCL_PC2
    pcl::toPCLPointCloud2(pubCloud, tmp_PCL);                       //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                      //PCL_PC2 -> sensor_msg_PC2
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
    return output;
}