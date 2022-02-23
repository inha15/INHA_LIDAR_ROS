#include "Lidar_declare.h"

using namespace std;

void ROI(const sensor_msgs::PointCloud2ConstPtr& scan){
    PCXYZI rawData;
    pcl::fromROSMsg(*scan,rawData);
    if(!switch_ROI){ goto jmp; } //ROI on/off
    for(unsigned int j = 0; j<rawData.points.size(); j++){     //actual ROI setting
        float *x = &rawData.points[j].x, *y = &rawData.points[j].y, *z = &rawData.points[j].z;
        if(*x > ROI_xMin && *x < ROI_xMax && *y > ROI_yMin && *y < ROI_yMax && *z > ROI_zMin && *z < ROI_zMax) continue;
        *x = *y = *z = 0;
    }
jmp:
    sensor_msgs::PointCloud2 output;                           //to output ROIdata formed PC2
    pub_process(rawData,output);
    pub_ROI.publish(output);
}

void UpSampling(PCXYZI& TotalCloud, PCXYZI::Ptr upsampledCloud){
    PCXYZI Data_for_voxel;
    pcl::MovingLeastSquares<PXYZI, PXYZI> filter;
    pcl::search::KdTree<PXYZI>::Ptr kdtree;
    
    copyPointCloud(TotalCloud, Data_for_voxel);
    filter.setInputCloud(Data_for_voxel.makeShared());
    filter.setSearchMethod(kdtree);
    filter.setSearchRadius(0.03);       // Use all neighbors in a radius of 3cm.
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<PXYZI, PXYZI>::SAMPLE_LOCAL_PLANE);
    filter.setUpsamplingRadius(0.03);   // Radius around each point, where the local plane will be sampled.
    filter.setUpsamplingStepSize(0.02); // Sampling step size. Bigger values will yield less (if any) new points.
    filter.process(*upsampledCloud);
    //cout << "PointCloud after upsampling has: " << upsampledCloud->points.size ()  << " data points." << endl; 
    
    //sensor_msgs::PointCloud2 output; 		          
    //pub_process(*upsampledCloud, output);             
    //pub_US.publish(output);
}

void DownSampling(PCXYZI& rawData, PCXYZI::Ptr downsampledCloud){ //Voxelization = DownSampling
    PCXYZI Data_for_voxel;
    pcl::VoxelGrid<PXYZI> vg;                            //declare voxel

    copyPointCloud(rawData, Data_for_voxel);            //rawData  ->  Data_for_voxel  ... just copy for modify
    vg.setInputCloud (Data_for_voxel.makeShared());     //Data_for_voxel  ->  vg space  ... deep copy , makeShared() return PC
    vg.setLeafSize (voxel_size_x, voxel_size_y, voxel_size_z);		        //voxel size setting(x,y,z)
    vg.filter (*downsampledCloud);                      //voxelized datas are included in downsampledCloud
    //cout << "PointCloud after downsampling has: " << downsampledCloud->points.size ()  << " data points." << endl; 

    //sensor_msgs::PointCloud2 output;
    //pub_process(*downsampledCloud, output);             
    //pub_DS.publish(output);   

}

void NoiseFiltering(PCXYZI::Ptr inputCloud, PCXYZI::Ptr outputCloud, string flag){
    pcl::StatisticalOutlierRemoval<PXYZI> tmp;
    tmp.setInputCloud(inputCloud);
    tmp.setMeanK(50);
    tmp.setStddevMulThresh(1.0);
    tmp.filter(*outputCloud);

    //sensor_msgs::PointCloud2 output;
    //pub_process(*outputCloud, output);
    //if(flag == "pre") pub_NF1.publish(output);
    //else if(flag == "post") pub_NF2.publish(output);
}

void Clustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud, string flag){
    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    tree->setInputCloud(inputCloud);                     // setting the KdTree

    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    pcl::EuclideanClusterExtraction<PXYZI> ec;            // clustering with Euclidean method
    ec.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    ec.setClusterTolerance(clustering_offset); 	         // dist between points ..  cur : 30cm
    ec.setMinClusterSize(MinClusterSize);				 // minSize the number of point for clustering
    ec.setMaxClusterSize(MaxClusterSize);	             // minSize the number of point for clustering
    ec.setSearchMethod(tree);				             // searching method : tree 
    ec.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    if(flag == "post") cout << "Number of clusters is equal to " << cluster_indices.size () << endl;    //return num of clusteringObj

    vector<pair<PXYZI,string>> sorted_OBJ; 
    //temp print middle point 
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        pair<float,float> x(9999,-9999); //first = min, second = max
        pair<float,float> y(9999,-9999); 
        pair<float,float> z(9999,-9999); 
    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            PXYZI pt = inputCloud->points[*pit];
            retCloud.push_back(pt);
            if(pt.x < x.first)      x.first = pt.x;
            if(pt.x > x.second)     x.second = pt.x;
            if(pt.y < y.first)      y.first = pt.y ;
            if(pt.y > y.second)     y.second = pt.y; 
            if(pt.z < z.first)      z.first = pt.z;
            if(pt.z > z.second)     z.second = pt.z;  
    	}
        if(flag == "pre"){
            PXYZI* tmp = new PXYZI(); //i dont know intensity initial format
            tmp->x = MidPt(x.first,x.second); tmp->y = MidPt(y.first,y.second); tmp->z = MidPt(z.first,z.second);
            pair<PXYZI,string> temp = make_pair(*tmp,send_msg_minmax(x.first, x.second, y.first, y.second));
            sorted_OBJ.push_back(temp);
        }
        else if(flag == "post"){
            PXYZI* tmp = new PXYZI();
            tmp->x = MidPt(x.first,x.second); tmp->y = MidPt(y.first,y.second); tmp->z = z.first; //z = min
            pair<PXYZI,string> temp = make_pair(*tmp,send_msg_minmax(x.first, x.second, y.first, y.second));
            sorted_OBJ.push_back(temp);
        }
    }
    sensor_msgs::PointCloud2 output; 
    pub_process(retCloud,output); 
    if(flag == "pre"){
        pub_Clu1.publish(output); 
    }
    else if(flag == "post"){
        cout << "------------------ DF & JF ------------------" << endl;
        FT.DY_filter(sorted_OBJ,switch_DY_filter);
        FT.jiwon_filter(sorted_OBJ,switch_jiwon_filter);
        print_OBJ(sorted_OBJ);
        msg_process(sorted_OBJ);
        pub_Clu2.publish(output);        
    }
}

void RanSaC(PCXYZI::Ptr upsampledCloud){
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    PCXYZI inlierPoints;
    PCXYZI inlierPoints_neg;
    pcl::SACSegmentation<PXYZI> seg;
    pcl::ExtractIndices<PXYZI> extract;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (ransac_distanceThreshold);             
    seg.setInputCloud(upsampledCloud); 
    seg.segment (*inliers, coefficients);

    pcl::copyPointCloud<PXYZI>(*upsampledCloud, *inliers, inlierPoints);    
    extract.setInputCloud (upsampledCloud);
    extract.setIndices (inliers);
    extract.setNegative (true);//false
    extract.filter (inlierPoints_neg);

    pcl::copyPointCloud(inlierPoints_neg, *upsampledCloud);
    sensor_msgs::PointCloud2 output; 
    pub_process(inlierPoints_neg, output); 
    pub_RS.publish(output); 
    pub_process(inlierPoints, output); 
    pub_GND.publish(output); 
}
