#include<potbot_pcl/Clustering3D.h>

void Clustering3DClass::__pcl2_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    header_ = msg->header;
    pcl::fromROSMsg(*msg, *cloud_raw_);
	ROS_INFO("==========");
    ROS_INFO("cloud_raw->points.size() = %d", cloud_raw_->points.size());

    __DownSampling();
    __Plane_removal();
    if (CLUSTERING_METHOD == "Euclidean")
    {
        __EuclideanClustering();
    }
    else if (CLUSTERING_METHOD == "SuperVoxel")
    {
        __SuperVoxelClustering();
    }
    
    
}

void Clustering3DClass::__param_callback(const potbot_pcl::ClusteringParamConfig& param, uint32_t level)
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