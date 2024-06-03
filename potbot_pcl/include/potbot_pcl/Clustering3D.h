#ifndef _H_CLUSTERING3D_
#define _H_CLUSTERING3D_

#include <random>
#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <potbot_lib/PCLClustering.h>
#include <potbot_msgs/ObstacleArray.h>
#include <potbot_msgs/ClusteringParamConfig.h>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

class Clustering3DClass{

    private:
        
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_pcl2_;
        
		ros::NodeHandle nhPub_;
        ros::Publisher pub_marker_, pub_pcl_0_, pub_pcl_1_, pub_obstacles_pcl_clustering_;

        tf2_ros::Buffer tf_buffer_;

        dynamic_reconfigure::Server<potbot_msgs::ClusteringParamConfig> *dsrv_;

        float   DownSampling_voxel_size_                = 0.01;

        double  PlaneRemoval_distance_threshold_        = 0.02;

        double  Euclidean_cluster_tolerance_            = 0.5;
        int     Euclidean_min_cluster_size_             = 100;

        float   Supervoxel_voxel_resolution_            = 0.008f;
        float   Supervoxel_seed_resolution_             = 0.1f;
        float   Supervoxel_color_importance_            = 0.2f;
        float   Supervoxel_spatial_importance_          = 0.4f;
        float   Supervoxel_normal_importance_           = 1.0f;

        std::string frame_id_global_ = "map", topic_pcl2_ = "depth/points", clustering_method_ = "Euclidean";
        
        void __pcl2_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void __param_callback(const potbot_msgs::ClusteringParamConfig& param, uint32_t level);
        
    public:
        Clustering3DClass(const std::string& name = "");
        ~Clustering3DClass();

};

#endif // _H_CLUSTERING3D_
