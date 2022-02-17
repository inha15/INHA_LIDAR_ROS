#ifndef LIDAR_DECLARE
#define LIDAR_DECLARE
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>   //noise filtering
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include "Lidar_pkg/Lidar_msg.h"  //include "패키지 명/메시지 파일 명.h"
#include "Lidar_filter.h"

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PCXYZI;
typedef pcl::PointXYZ PXYZ;

ros::Publisher pub_ROI;     //ROI
ros::Publisher pub_DS;      //downsampling
ros::Publisher pub_US;      //upsampling
ros::Publisher pub_NF1;     //nosiefiltering1
ros::Publisher pub_Clu1;    //clustering1
ros::Publisher pub_RS;      //ransac
ros::Publisher pub_GND;     //ground
ros::Publisher pub_NF2;     //nosiefiltering2
ros::Publisher pub_Clu2;    //clustering2
//ros::Publisher OUT_MSG;     //out message
ros::Publisher pub_msg;

class parameter{
public:
    double REMOVE_FACTOR;
    float ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax;
    float Ransac_Z_ROI;
    float clustering_offset;
    int MinClusterSize, MaxClusterSize;
    double ransac_distanceThreshold; //함수 : double형 parameter
    bool preNoiseFiltering;
    bool postNoiseFiltering;
    bool jiwon_filter;
    bool DY_filter;
    bool ROI;
    bool DownSampling;
    bool pre_Clustering;
    bool UpSampling;
    bool RanSaC;
    bool post_Clustering;
};
parameter P;


#endif
