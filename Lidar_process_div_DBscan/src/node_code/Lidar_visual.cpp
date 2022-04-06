#include <Lidar_process_div/Lidar_declare.h>

using namespace std;

pcl::visualization::PCLVisualizer* viewer;

void visual_process(const sensor_msgs::PointCloud2ConstPtr& aft_preClustering){
    pcl::PointCloud<pcl::PointXYZI>::Ptr to_show(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*aft_preClustering, *to_show);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> h1(to_show, 255, 255, 255);
    // viewer->addPointCloud(to_show, h1, "h1");
    // viewer->updatePointCloud(to_show, h1, "h1");
    if (viewer->contains("h1")) viewer->updatePointCloud(to_show, h1, "h1");
    else viewer->addPointCloud(to_show, h1, "h1");   
}

int main(int argc, char** argv){
	ros::init(argc, argv, "lidar_visual"); //node name 
	ros::NodeHandle nh;         //nodehandle
    /*
    viewer = new pcl::visualization::PCLVisualizer("LiDAR Point Cloud Viewer");
    viewer->setBackgroundColor (0.1, 0.1, 0.15);
    viewer->initCameraParameters();
    viewer->setCameraPosition(-5.0, 5.0, 2.0,    6.0, -3.0, 0.0,   0.0, 0.0, 1.0);
    viewer->addCoordinateSystem (0.3f);    

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/4_velodyne_points_DBscan", 100, visual_process);   
    
	ros::spin();
    
    while(!viewer->wasStopped()) {
        ros::spinOnce();    
        viewer->spinOnce();
    }

    delete viewer;
    */
}