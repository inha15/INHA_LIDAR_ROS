#include "Lidar_declare.h"


using namespace std;

//func
template<typename T>
void pub_process(T&, sensor_msgs::PointCloud2&);
void ROI(PCXYZI&);
void UpSampling(PCXYZ&, PCXYZ::Ptr);
void DownSampling(PCXYZI&, PCXYZ::Ptr);
void NoiseFiltering(PCXYZ::Ptr, PCXYZ::Ptr, string);
void Clustering(PCXYZ::Ptr, PCXYZ&, string);
void post_Clustering(PCXYZ::Ptr, PCXYZ&);
void RanSaC(PCXYZ::Ptr);
void whole_processing(const sensor_msgs::PointCloud2ConstPtr&);


void whole_processing(const sensor_msgs::PointCloud2ConstPtr& scan){ //scan = raw data
    PCXYZI rawData;
    pcl::fromROSMsg(*scan,rawData); //const value is not modified. sensor_msgs PC2 -> PC 
    //-----------ROI-----------
    if( P.ROI ) ROI(rawData); //call ROI setting func
    //-----------DownSampling-----------
    PCXYZ::Ptr downsampledCloud (new PCXYZ); //saving spcae for downsampledData
    if( P.DownSampling ) DownSampling(rawData,downsampledCloud); //call DownSampling func
    else copyPointCloud(rawData,*downsampledCloud);
    //-----------pre_NoiseFiltering-----------
    PCXYZ::Ptr prefilteredCloud (new PCXYZ); //saving spcae for filteredData
    if( P.preNoiseFiltering ) NoiseFiltering(downsampledCloud,prefilteredCloud, "pre"); //call NoiseFiltering func  ////  current : off
    else prefilteredCloud = downsampledCloud;
    //-----------pre_Clustering-----------
    PCXYZ TotalCloud; //possible to put in func, but... for the enpandability
    if( P.pre_Clustering ) Clustering(prefilteredCloud,TotalCloud,"pre");
    else TotalCloud = *prefilteredCloud;
    //-----------UpSampling-----------
    PCXYZ::Ptr upsampledCloud (new PCXYZ);
    if( P.UpSampling ) UpSampling(TotalCloud,upsampledCloud);
    else *upsampledCloud = TotalCloud;
    //----------ransac----------
    if( P.RanSaC ) RanSaC(upsampledCloud);
    //-----------post_NoiseFiltering-----------
    PCXYZ::Ptr postfilteredCloud (new PCXYZ); //saving spcae for filteredData
    if( P.postNoiseFiltering ) NoiseFiltering(upsampledCloud,postfilteredCloud, "post"); //call NoiseFiltering func  ////  current : on
    else postfilteredCloud = upsampledCloud;
    //-----------post_clustering------------
    PCXYZ Fin_Cloud;
    if( P.post_Clustering ) Clustering(postfilteredCloud,Fin_Cloud,"post");
    else Fin_Cloud = *postfilteredCloud;
    FPS1.update();
}

template<typename T>
void pub_process(T& input, sensor_msgs::PointCloud2& output){
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(input, tmp_PCL);                     //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.frame_id = "velodyne";
}

void ROI(PCXYZI& rawData){
    for(unsigned int j = 0; j<rawData.points.size(); j++){     //actual ROI setting
        float *x = &rawData.points[j].x, *y = &rawData.points[j].y, *z = &rawData.points[j].z;
        if(*x > P.ROI_xMin && *x < P.ROI_xMax && *y > P.ROI_yMin && *y < P.ROI_yMax && *z > P.ROI_zMin && *z < P.ROI_zMax) continue;
        *x = *y = *z = 0;
    }
    sensor_msgs::PointCloud2 output;                           //to output ROIdata formed PC2
    pub_process(rawData,output);
    pub_ROI.publish(output);
}

void UpSampling(PCXYZ& TotalCloud, PCXYZ::Ptr upsampledCloud){
    PCXYZ Data_for_voxel;
    pcl::MovingLeastSquares<PXYZ, PXYZ> filter;
    pcl::search::KdTree<PXYZ>::Ptr kdtree;
    
    copyPointCloud(TotalCloud, Data_for_voxel);
    filter.setInputCloud(Data_for_voxel.makeShared());
    filter.setSearchMethod(kdtree);
    filter.setSearchRadius(0.03);       // Use all neighbors in a radius of 3cm.
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<PXYZ, PXYZ>::SAMPLE_LOCAL_PLANE);
    filter.setUpsamplingRadius(0.03);   // Radius around each point, where the local plane will be sampled.
    filter.setUpsamplingStepSize(0.02); // Sampling step size. Bigger values will yield less (if any) new points.
    filter.process(*upsampledCloud);
    //cout << "PointCloud after upsampling has: " << upsampledCloud->points.size ()  << " data points." << endl; 
    
    sensor_msgs::PointCloud2 output; 		          
    pub_process(*upsampledCloud, output);             
    pub_US.publish(output);
}
//Voxelization = DownSampling
void DownSampling(PCXYZI& rawData, PCXYZ::Ptr downsampledCloud){ 
    PCXYZ Data_for_voxel;
    pcl::VoxelGrid<PXYZ> vg;                            //declare voxel

    copyPointCloud(rawData, Data_for_voxel);            //rawData  ->  Data_for_voxel  ... just copy for modify
    vg.setInputCloud (Data_for_voxel.makeShared());     //Data_for_voxel  ->  vg space  ... deep copy , makeShared() return PC
    vg.setLeafSize (P.voxel_size_x, P.voxel_size_y, P.voxel_size_z);		        //voxel size setting(x,y,z)
    vg.filter (*downsampledCloud);                      //voxelized datas are included in downsampledCloud
    //cout << "PointCloud after downsampling has: " << downsampledCloud->points.size ()  << " data points." << endl; 

    sensor_msgs::PointCloud2 output;
    pub_process(*downsampledCloud, output);             
    pub_DS.publish(output);   
}

void NoiseFiltering(PCXYZ::Ptr inputCloud, PCXYZ::Ptr outputCloud, string flag){
    pcl::StatisticalOutlierRemoval<PXYZ> tmp;
    tmp.setInputCloud(inputCloud);
    tmp.setMeanK(50);
    tmp.setStddevMulThresh(1.0);
    tmp.filter(*outputCloud);

    sensor_msgs::PointCloud2 output;
    pub_process(*outputCloud, output);
    if(flag == "pre") pub_NF1.publish(output);
    else if(flag == "post") pub_NF2.publish(output);
}

void Clustering(PCXYZ::Ptr inputCloud, PCXYZ& retCloud, string flag){
    pcl::search::KdTree<PXYZ>::Ptr tree (new pcl::search::KdTree<PXYZ>);  // Creating the KdTree for searching PC
    tree->setInputCloud(inputCloud);                     // setting the KdTree

    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    pcl::EuclideanClusterExtraction<PXYZ> ec;            // clustering with Euclidean method
    ec.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    ec.setClusterTolerance(P.clustering_offset); 	     // dist between points ..  cur : 30cm
    ec.setMinClusterSize(P.MinClusterSize);				 // minSize the number of point for clustering
    ec.setMaxClusterSize(P.MaxClusterSize);	             // minSize the number of point for clustering
    ec.setSearchMethod(tree);				             // searching method : tree 
    ec.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    if(flag == "post") cout << "Number of clusters is equal to " << cluster_indices.size () << endl;    //return num of clusteringObj

    vector<pair<PXYZ,string>> sorted_OBJ; 
    //temp print middle point 
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        pair<float,float> x(9999,-9999); //first = min, second = max
        pair<float,float> y(9999,-9999); 
        pair<float,float> z(9999,-9999); 
    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            PXYZ pt = inputCloud->points[*pit];
            retCloud.push_back(pt);
            if(pt.x < x.first)      x.first = pt.x;
            if(pt.x > x.second)     x.second = pt.x;
            if(pt.y < y.first)      y.first = pt.y ;
            if(pt.y > y.second)     y.second = pt.y; 
            if(pt.z < z.first)      z.first = pt.z;
            if(pt.z > z.second)     z.second = pt.z;  
    	}
        if(flag == "pre"){
            PXYZ* tmp = new PXYZ(MidPt(x.first,x.second), MidPt(y.first,y.second), MidPt(z.first,z.second));
            pair<PXYZ,string> temp = make_pair(*tmp,send_msg_minmax(x.first, x.second, y.first, y.second));
            sorted_OBJ.push_back(temp);
        }
        else if(flag == "post"){
            PXYZ* tmp = new PXYZ(MidPt(x.first,x.second), MidPt(y.first,y.second), z.first); //z = min
            pair<PXYZ,string> temp = make_pair(*tmp,send_msg_minmax(x.first, x.second, y.first, y.second));
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
        FT.DY_filter(sorted_OBJ,P.DY_filter);
        FT.jiwon_filter(sorted_OBJ,P.jiwon_filter);
        print_OBJ(sorted_OBJ);
        msg_process(sorted_OBJ);
        pub_Clu2.publish(output);        
    }
}

void RanSaC(PCXYZ::Ptr upsampledCloud){
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    PCXYZ inlierPoints;
    PCXYZ inlierPoints_neg;
    pcl::SACSegmentation<PXYZ> seg;
    pcl::ExtractIndices<PXYZ> extract;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (P.ransac_distanceThreshold);             
    seg.setInputCloud(upsampledCloud); 
    seg.segment (*inliers, coefficients);

    pcl::copyPointCloud<PXYZ>(*upsampledCloud, *inliers, inlierPoints);    
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

int main(int argc, char** argv){
	ros::init(argc, argv, "processing_LidarData"); //node name 
	ros::NodeHandle nh;         //nodehandle
    set_param(nh,P);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, whole_processing);
    pub_ROI = nh.advertise<sensor_msgs::PointCloud2> ("/1_velodyne_points_ROI", 100);
    pub_DS = nh.advertise<sensor_msgs::PointCloud2> ("/2_velodyne_points_downsampled", 100);//downsampling
    pub_US = nh.advertise<sensor_msgs::PointCloud2> ("/5_velodyne_points_upsampled", 100);//upsampling
    pub_NF1 = nh.advertise<sensor_msgs::PointCloud2> ("/3_velodyne_points_preNoisefiltered", 100);
    pub_Clu1 = nh.advertise<sensor_msgs::PointCloud2> ("/4_velodyne_points_preClustered", 1);  
    pub_RS = nh.advertise<sensor_msgs::PointCloud2> ("/6_1_velodyne_points_ransac", 100);  
    pub_GND = nh.advertise<sensor_msgs::PointCloud2> ("/6_2_velodyne_points_ground", 100);
    pub_NF2 = nh.advertise<sensor_msgs::PointCloud2> ("/7_velodyne_points_postNoisefiltered", 100);    
    pub_Clu2 = nh.advertise<sensor_msgs::PointCloud2> ("/8_velodyne_points_postClustered", 1);
    pub_msg = nh.advertise<std_msgs::String> ("/Lidar_msg",1); 

    //OUT_MSG = nh.advertise<Lidar_pkg::Lidar_msg> ("/lidar_detected_object", 1);
    //<패키지 명/메시지 파일 명>

	ros::spin();
}

