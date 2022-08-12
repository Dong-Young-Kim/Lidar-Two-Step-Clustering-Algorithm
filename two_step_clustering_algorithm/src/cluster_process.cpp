#include <two_step_clustering_algorithm/cluster_process.h>


TSC::TwoStepClustering::TwoStepClustering(){
    nh.getParam("two_step_clustering_algorithm", switchMultiLiDAR);
    switch (switchMultiLiDAR){
    case true:
        InitNodeMultiLiDAR();
        break;
    case false:
        InitNode();
        break;
    }
}

void TransformPC(pcl::PointCloud<pcl::PointXYZI>& inputSource){
    /* Reminder: how transformation matrices work :

            |-------> This column is the translation
        | 1 0 0 x |  \
        | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
        | 0 0 1 z |  /
        | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

        METHOD #1: Using a Matrix4f
        This is the "manual" method, perfect to understand but error prone !
    */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI/4; // The angle of rotation in radians
    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = 2.5;

    // Print the transformation
    printf ("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /*  METHOD #2: Using a Affine3f
        This method is easier and less error prone
    */
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (inputSource, *transformed_cloud, transform_2);
}

void TSC::TwoStepClustering::InitNode(){
    pub_first = nh.advertise<sensor_msgs::PointCloud2> ("two_step_clustering_first_clusterd", 1);
    sub = nh.subscribe<sensor_msgs::PointCloud2> ("/2_1_velodyne_points_ransac", 1, &TSC::TwoStepClustering::SubscribeCallback, this);
}

void TSC::TwoStepClustering::InitNodeMultiLiDAR(){
    pub_first = nh.advertise<sensor_msgs::PointCloud2> ("two_step_clustering_first_clusterd", 1);
    multiLiDARProcessFlag = std::make_tuple(0,0,0,0,1);
	sub_multi_lidar1 = nh.subscribe<sensor_msgs::PointCloud2> ("/lidar1/velodyne_points", 1, &TSC::TwoStepClustering::SubscribeCallbackMulti_1, this);
	sub_multi_lidar2 = nh.subscribe<sensor_msgs::PointCloud2> ("/lidar2/velodyne_points", 1, &TSC::TwoStepClustering::SubscribeCallbackMulti_2, this);
}

void TSC::TwoStepClustering::AllClear(){
    firstClustered          .clear();
    seconedClustered        .clear();
    firstClusterIndices     .clear();
    secondClusterIndices    .clear();
    objs                    .clear();
}

void TSC::TwoStepClustering::AllClearMulti(){

    // firstClustered.clear();
    // seconedClustered.clear();
    // first_cluster_indices.clear();
    // second_cluster_indices.clear();
    // objs.clear();
}

void TSC::TwoStepClustering::SubscribeCallback(const sensor_msgs::PointCloud2ConstPtr& input_data){
    std::cout << "data input\n";
    AllClear();
    pcl::fromROSMsg(*input_data, inputCloud);

    FirstClustering(inputCloud, firstClusterIndices);
    AfterFirstClustering(inputCloud, firstClusterIndices, firstClustered, objs);
    pub_first.publish(this->Publish(this->firstClustered));

    //SecondClustering();

}

void TSC::TwoStepClustering::SubscribeCallbackMulti_1(const sensor_msgs::PointCloud2ConstPtr& input_data){
    if(std::get<4>(multiLiDARProcessFlag)) AllClearMulti();
    std::cout << "data input from first LiDAR\n";
    pcl::fromROSMsg(*input_data, inputCloudMulti_1);
    FirstClustering(inputCloudMulti_1, firstClusterIndicesMulti_1);
    AfterFirstClustering(inputCloudMulti_1, firstClusterIndicesMulti_1, firstClusteredMulti_1, objsMulti_1);
    //SecondClustering();

}

void TSC::TwoStepClustering::Run(){
    while(ros::ok()){
        ros::spinOnce();
    }
}

void TSC::TwoStepClustering::FirstClustering(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices>& output){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    *cloud = input;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    FastClustering<pcl::PointXYZI> fc;
    fc.setInputCloud(cloud);
    fc.setSearchMethod(tree);
    fc.setMinClusterSize(1);
    fc.setMaxClusterSize(1000);
    fc.setClusterToleranceAngle(0.5);
    fc.extract(output);
}

void TSC::TwoStepClustering::SecondClustering(){

}

void TSC::TwoStepClustering::AfterFirstClustering(pcl::PointCloud<pcl::PointXYZI>& inputPC, std::vector<pcl::PointIndices>& input, 
                                                  pcl::PointCloud<pcl::PointXYZI>& outputPC, std::vector<struct objInfo>& outputObjs){
    int intensityValue = 0;
    for (std::vector<pcl::PointIndices>::iterator it = input.begin(); it != input.end(); ++it, intensityValue++){

        std::pair<float,float> x(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); //first = min, second = max
        std::pair<float,float> y(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
        std::pair<float,float> z(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());

    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            pcl::PointXYZI pt = inputPC.points[*pit];
            pt.intensity = intensityValue % 10;
            outputPC.push_back(pt);
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
        outputObjs.push_back(tmp_obj);
    }
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