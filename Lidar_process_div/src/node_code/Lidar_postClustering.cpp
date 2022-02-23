#include "Lidar_declare.h"

using namespace std;

void postClustering_process(const sensor_msgs::PointCloud2ConstPtr& aft_ransac){
    PCXYZI tmp;
    pcl::fromROSMsg(*aft_ransac,tmp);
    PCXYZI::Ptr upsampledCloud (new PCXYZI);;
    copyPointCloud(tmp,*upsampledCloud);
    //-----------post_NoiseFiltering-----------
    PCXYZI::Ptr postfilteredCloud (new PCXYZI); //saving spcae for filteredData
    if( switch_postNoiseFiltering ) NoiseFiltering(upsampledCloud,postfilteredCloud, "post"); //call NoiseFiltering func  ////  current : on
    else postfilteredCloud = upsampledCloud;
    //-----------post_clustering------------
    PCXYZI Fin_Cloud;
    if( switch_post_Clustering ) Clustering(postfilteredCloud,Fin_Cloud,"post");
    else{
        Fin_Cloud = *postfilteredCloud;
        sensor_msgs::PointCloud2 output; 
        pub_process(Fin_Cloud,output);
        pub_Clu2.publish(output);        
    } 
    FPS1.update();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "postClustering"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/postClustering_node/switch_postNoiseFiltering", switch_postNoiseFiltering);
    nh.getParam("/postClustering_node/switch_post_Clustering", switch_post_Clustering);
    nh.getParam("/postClustering_node/switch_jiwon_filter", switch_jiwon_filter);
    nh.getParam("/postClustering_node/switch_DY_filter", switch_DY_filter);
    nh.getParam("/postClustering_node/clustering_offset", clustering_offset);
    nh.getParam("/postClustering_node/MinClusterSize", MinClusterSize);
    nh.getParam("/postClustering_node/MaxClusterSize", MaxClusterSize);
    nh.getParam("/postClustering_node/Ransac_Z_ROI", Ransac_Z_ROI);  //DY_filter param
    nh.getParam("/postClustering_node/REMOVE_FACTOR", REMOVE_FACTOR);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/3_1_velodyne_points_ransac", 100, postClustering_process);
    pub_Clu2 = nh.advertise<sensor_msgs::PointCloud2> ("/4_velodyne_points_postClustered", 1);
    pub_msg = nh.advertise<std_msgs::String> ("/Lidar_msg",1); 

    //OUT_MSG = nh.advertise<Lidar_pkg::Lidar_msg> ("/lidar_detected_object", 1);
    //<패키지 명/메시지 파일 명>
	ros::spin();
}
