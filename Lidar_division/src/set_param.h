#ifndef LIDAR_PARAM
#define LIDAR_PARAM
#include "Lidar_declare.h"

using namespace std;

template<typename T>
void set_param(ros::NodeHandle& nh, T& P){
    nh.getParam("/code_executed/REMOVE_FACTOR", P.REMOVE_FACTOR);
    nh.getParam("/code_executed/voxel_size_x", P.voxel_size_x);
    nh.getParam("/code_executed/voxel_size_y", P.voxel_size_y);
    nh.getParam("/code_executed/voxel_size_z", P.voxel_size_z);
    nh.getParam("/code_executed/ROI_xMin", P.ROI_xMin);
    nh.getParam("/code_executed/ROI_xMax", P.ROI_xMax);
    nh.getParam("/code_executed/ROI_yMin", P.ROI_yMin);
    nh.getParam("/code_executed/ROI_yMax", P.ROI_yMax);
    nh.getParam("/code_executed/ROI_zMin", P.ROI_zMin);
    nh.getParam("/code_executed/ROI_zMax", P.ROI_zMax);
    nh.getParam("/code_executed/clustering_offset", P.clustering_offset);
    nh.getParam("/code_executed/MinClusterSize", P.MinClusterSize);
    nh.getParam("/code_executed/MaxClusterSize", P.MaxClusterSize);
    nh.getParam("/code_executed/ransac_distanceThreshold", P.ransac_distanceThreshold);
    nh.getParam("/code_executed/Ransac_Z_ROI", P.Ransac_Z_ROI);
    nh.getParam("/code_executed/preNoiseFiltering", P.preNoiseFiltering);
    nh.getParam("/code_executed/ROI", P.ROI);
    nh.getParam("/code_executed/DownSampling", P.DownSampling);
    nh.getParam("/code_executed/pre_Clustering", P.pre_Clustering);
    nh.getParam("/code_executed/UpSampling", P.UpSampling);
    nh.getParam("/code_executed/RanSaC", P.RanSaC);
    nh.getParam("/code_executed/postNoiseFiltering", P.postNoiseFiltering);
    nh.getParam("/code_executed/post_Clustering", P.post_Clustering);
    nh.getParam("/code_executed/jiwon_filter", P.jiwon_filter);
    nh.getParam("/code_executed/DY_filter", P.DY_filter);    
}


#endif