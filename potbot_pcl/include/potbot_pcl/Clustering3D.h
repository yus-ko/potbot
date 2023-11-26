#include <random>

#include <ros/ros.h>
#include <potbot_lib/Utility.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <potbot_pcl/ClusteringParamConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

//クラスの定義
class Clustering3DClass{

    private:

        // tf::TransformListener tflistener;
        // tf2_ros::Buffer tf_buffer_;
        
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub_pcl2_;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_marker_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ {new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ {new pcl::PointCloud<pcl::PointXYZ>};

        std_msgs::Header header_;

        dynamic_reconfigure::Server<potbot_pcl::ClusteringParamConfig> server_;
  	    dynamic_reconfigure::Server<potbot_pcl::ClusteringParamConfig>::CallbackType f_;

        float   DownSampling_voxel_size_                = 0.01;

        double  PlaneRemoval_distance_threshold_        = 0.02;

        double  Euclidean_cluster_tolerance_            = 0.5;
        int     Euclidean_min_cluster_size_             = 100;

        float   Supervoxel_voxel_resolution_            = 0.008f;
        float   Supervoxel_seed_resolution_             = 0.1f;
        float   Supervoxel_color_importance_            = 0.2f;
        float   Supervoxel_spatial_importance_          = 0.4f;
        float   Supervoxel_normal_importance_           = 1.0f;

        std::string TOPIC_PCL2, CLUSTERING_METHOD = "Euclidean";
        // std::string PATH_PLANNING_METHOD;
        // double TARGET_POSITION_X;
        
        void __pcl2_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void __param_callback(const potbot_pcl::ClusteringParamConfig& param, uint32_t level);

        void __DownSampling();
        void __Plane_removal();

        void __EuclideanClustering();
        void __SuperVoxelClustering();
        
    public:
        Clustering3DClass();
        ~Clustering3DClass();
        void setLaunchParam();//launchファイルから書き込み
        
        void mainloop();

        // void publishPathPlan();
};
