#include<potbot_pcl/Clustering3D.h>

Clustering3DClass::Clustering3DClass()
{

	ros::NodeHandle n("~");
    n.getParam("topic_pcl2",            topic_pcl2_);
    n.getParam("clustering_method",     clustering_method_);

	sub_pcl2_	= nhSub_.subscribe(topic_pcl2_,1,&Clustering3DClass::__pcl2_callback,this);

	pub_marker_ = nhPub_.advertise<visualization_msgs::MarkerArray>("filtered_pcl", 1);
	pub_pcl_0_ = nhPub_.advertise<sensor_msgs::PointCloud2>("debug/pcl/DownSampling", 1);
	pub_pcl_1_ = nhPub_.advertise<sensor_msgs::PointCloud2>("debug/pcl/Plane_removal", 1);

	f_ = boost::bind(&Clustering3DClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

}
Clustering3DClass::~Clustering3DClass(){
}

void Clustering3DClass::__pcl2_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    header_ = msg->header;
    pcl::fromROSMsg(*msg, *cloud_raw_);
	// ROS_INFO("==========");
    // ROS_INFO("cloud_raw->points.size() = %d", cloud_raw_->points.size());

    __DownSampling();
    __Plane_removal();
    if (clustering_method_ == "Euclidean")
    {
        __EuclideanClustering();
    }
    else if (clustering_method_ == "SuperVoxel")
    {
        __SuperVoxelClustering();
    }
    
}

void Clustering3DClass::__param_callback(const potbot_msgs::ClusteringParamConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    DownSampling_voxel_size_            = param.DownSampling_voxel_size;

    PlaneRemoval_distance_threshold_    = param.PlaneRemoval_distance_threshold;

    Euclidean_cluster_tolerance_        = param.Euclidean_cluster_tolerance;
    Euclidean_min_cluster_size_         = param.Euclidean_min_cluster_size;

    Supervoxel_voxel_resolution_        = param.Supervoxel_voxel_resolution;
    Supervoxel_seed_resolution_         = param.Supervoxel_seed_resolution;
    Supervoxel_color_importance_        = param.Supervoxel_color_importance;
    Supervoxel_spatial_importance_      = param.Supervoxel_spatial_importance;
    Supervoxel_normal_importance_       = param.Supervoxel_normal_importance;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_cl3d");

    Clustering3DClass cl3d;
	ros::spin();

	return 0;
}