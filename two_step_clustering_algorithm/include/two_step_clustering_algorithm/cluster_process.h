#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <tuple>
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
#include <pcl/common/transforms.h>

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

    protected:
        ros::NodeHandle     nh;

        ros::Subscriber     sub_multi_lidar1;
        ros::Subscriber     sub_multi_lidar2;
        ros::Publisher      pub_RotetedAndMergedMulti;
        ros::Publisher      pub_firstClusteredMulti_1;
        ros::Publisher      pub_firstClusteredMulti_2;


        pcl::PointCloud<pcl::PointXYZI>     inputCloudMulti_1;
        pcl::PointCloud<pcl::PointXYZI>     inputCloudMulti_2;

        pcl::PointCloud<pcl::PointXYZI>     rotatedCloudMulti_1;
        pcl::PointCloud<pcl::PointXYZI>     rotatedCloudMulti_2;
        pcl::PointCloud<pcl::PointXYZI>     rotatedCloudMultiMerged;


        pcl::PointCloud<pcl::PointXYZI>     firstClusteredMulti_1;
        pcl::PointCloud<pcl::PointXYZI>     firstClusteredMulti_2;

        std::vector<struct objInfo>         objsMulti_1;
        std::vector<struct objInfo>         objsMulti_2;


        std::vector<pcl::PointIndices>      firstClusterIndicesMulti_1;
        std::vector<pcl::PointIndices>      firstClusterIndicesMulti_2;


    
        bool switchMultiLiDAR = false;
        float transform_factor;
        std::tuple<bool, bool, bool, bool, bool> multiLiDARProcessFlag;

        void TransformPC                (pcl::PointCloud<pcl::PointXYZI>& inputSource, pcl::PointCloud<pcl::PointXYZI>& outputResult,
                                         float thetaRotation, float meterTransform);
        void MergePC                    ();

        void InitNode                   ();
        void InitNodeMultiLiDAR         ();
        void SubscribeCallback          (const sensor_msgs::PointCloud2ConstPtr& input_data);       //edit overall process here
        void SubscribeCallbackMulti_1   (const sensor_msgs::PointCloud2ConstPtr& input_data);       //edit overall process here
        void SubscribeCallbackMulti_2   (const sensor_msgs::PointCloud2ConstPtr& input_data);       //edit overall process here

        void AllClear                   ();
        void AllClearMulti              ();
        void FirstClustering            (pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices>& output);
        void SecondClustering           ();
        void AfterFirstClustering       (pcl::PointCloud<pcl::PointXYZI>& inputPC, std::vector<pcl::PointIndices>& input, 
                                         pcl::PointCloud<pcl::PointXYZI>& outputPC, std::vector<struct objInfo>& outputObjs);
        sensor_msgs::PointCloud2 Publish(pcl::PointCloud<pcl::PointXYZI> pubCloud);

    private:
        ros::Subscriber     sub;
        ros::Publisher      pub_firstClustered;
        ros::Publisher      pub_secondClustered;

        pcl::PointCloud<pcl::PointXYZI>     inputCloud;       
        pcl::PointCloud<pcl::PointXYZI>     firstClustered;
        pcl::PointCloud<pcl::PointXYZI>     seconedClustered;
        std::vector<struct objInfo>         objs;
        std::vector<pcl::PointIndices>      firstClusterIndices;
        std::vector<pcl::PointIndices>      secondClusterIndices;



    };

    class TwoStepClusteringMultiLiDAR : public TSC::TwoStepClustering{
    public:
    protected:
    private:

    };

}