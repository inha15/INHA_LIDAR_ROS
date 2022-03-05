#include <Lidar_process_div/Lidar_declare.h>

using namespace std;

void ransac_process(const sensor_msgs::PointCloud2ConstPtr& aft_preClustering){
    PCXYZI tmp;
    pcl::fromROSMsg(*aft_preClustering,tmp);
    PCXYZI TotalCloud;
    copyPointCloud(tmp,TotalCloud);
    //-----------UpSampling-----------
    PCXYZI::Ptr upsampledCloud (new PCXYZI);
    if( switch_UpSampling ) UpSampling(TotalCloud,upsampledCloud);
    else *upsampledCloud = TotalCloud;
    //----------ransac----------
    if( switch_RanSaC ) RanSaC(upsampledCloud);
    else{
        sensor_msgs::PointCloud2 output; 
        pub_process(*upsampledCloud, output); 
        pub_RS.publish(output); 
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ransac"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/ransac_node/switch_UpSampling", switch_UpSampling);
    nh.getParam("/ransac_node/switch_RanSaC", switch_RanSaC);
    nh.getParam("/ransac_node/ransac_distanceThreshold", ransac_distanceThreshold);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/2_velodyne_points_preClustered", 100, ransac_process);
    pub_RS = nh.advertise<sensor_msgs::PointCloud2> ("/3_1_velodyne_points_ransac", 100);
    pub_GND = nh.advertise<sensor_msgs::PointCloud2> ("/3_2_velodyne_points_ground", 100);
    
	ros::spin();
}