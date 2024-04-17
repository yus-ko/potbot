#include<potbot_pcl/Clustering3D.h>

Clustering3DClass::Clustering3DClass()
{

	ros::NodeHandle n("~");
    n.getParam("frame_id_global",       frame_id_global_);
    n.getParam("topic_pcl2",            topic_pcl2_);
    n.getParam("clustering_method",     clustering_method_);

	sub_pcl2_	= nhSub_.subscribe(topic_pcl2_,1,&Clustering3DClass::__pcl2_callback,this);

	pub_marker_ = nhPub_.advertise<visualization_msgs::MarkerArray>("filtered_pcl", 1);
	pub_pcl_0_ = nhPub_.advertise<sensor_msgs::PointCloud2>("debug/pcl/DownSampling", 1);
	pub_pcl_1_ = nhPub_.advertise<sensor_msgs::PointCloud2>("debug/pcl/Plane_removal", 1);
    pub_obstacles_pcl_clustering_ = nhPub_.advertise<potbot_msgs::ObstacleArray>("obstacle/pcl/clustering", 1);

	f_ = boost::bind(&Clustering3DClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

    static tf2_ros::TransformListener tfListener(tf_buffer_);

}
Clustering3DClass::~Clustering3DClass(){
}

void Clustering3DClass::__pcl2_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // ros::Time start_time = ros::Time::now();

    sensor_msgs::PointCloud2 cloud_ros_down_sampling, cloud_ros_plane_removal;
    visualization_msgs::MarkerArray cloud_markers;
    potbot_msgs::ObstacleArray cloud_obstacle_array;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::fromROSMsg(*msg, *cloud_raw);

    if (clustering_method_ == "Euclidean")
    {
        potbot_lib::PCLClustering pcl_clustering;

        pcl_clustering.set_down_sampling_voxel_size(DownSampling_voxel_size_);
        pcl_clustering.set_plane_removal_distance_threshold(PlaneRemoval_distance_threshold_);

        pcl_clustering.set_clusters(cloud_raw);

        pcl_clustering.down_sampling();
        pcl_clustering.get_clusters(cloud_ros_down_sampling);

        pcl_clustering.plane_removal();
        pcl_clustering.get_clusters(cloud_ros_plane_removal);

        pcl_clustering.set_euclidean_min_cluster_size(Euclidean_min_cluster_size_);
        pcl_clustering.set_euclidean_cluster_tolerance(Euclidean_cluster_tolerance_);
        pcl_clustering.euclidean_clustering();
        
        pcl_clustering.get_clusters(cloud_markers);
        pcl_clustering.get_clusters(cloud_obstacle_array);
    }
    else if (clustering_method_ == "SuperVoxel")
    {
        potbot_lib::PCLSuperVoxel pcl_clustering;

        pcl_clustering.set_down_sampling_voxel_size(DownSampling_voxel_size_);
        pcl_clustering.set_plane_removal_distance_threshold(PlaneRemoval_distance_threshold_);

        pcl_clustering.set_clusters(cloud_raw);

        pcl_clustering.down_sampling();
        pcl_clustering.get_clusters(cloud_ros_down_sampling);

        pcl_clustering.plane_removal();
        pcl_clustering.get_clusters(cloud_ros_plane_removal);

        pcl_clustering.set_supervoxel_color_importance(Supervoxel_color_importance_);
        pcl_clustering.set_supervoxel_normal_importance(Supervoxel_normal_importance_);
        pcl_clustering.set_supervoxel_seed_resolution(Supervoxel_seed_resolution_);
        pcl_clustering.set_supervoxel_spatial_importance(Supervoxel_spatial_importance_);
        pcl_clustering.set_supervoxel_voxel_resolution(Supervoxel_voxel_resolution_);
        pcl_clustering.supervoxel_clustering();

        pcl_clustering.get_clusters(cloud_markers);
        pcl_clustering.get_clusters(cloud_obstacle_array);
    }

    //1時刻前のクラスタからの追跡
    static potbot_msgs::ObstacleArray cloud_obstacle_array_pre;
    potbot_lib::utility::associate_obstacle(cloud_obstacle_array, cloud_obstacle_array_pre, tf_buffer_);

    //クラスタをワールド座標系に変換して1時刻先の追跡用データにする
    // cloud_obstacle_array.header = msg->header;
    cloud_obstacle_array.header.stamp = ros::Time::now();
    // std::cout<< cloud_obstacle_array.header <<std::endl;
    for (auto& obs: cloud_obstacle_array.data)
    {
        // obs.header = msg->header;
        obs.header.stamp = cloud_obstacle_array.header.stamp;
        // std::cout<< obs.header <<std::endl;
    }
    potbot_lib::utility::get_tf(tf_buffer_, cloud_obstacle_array, frame_id_global_, cloud_obstacle_array_pre);

    pub_marker_.publish(cloud_markers);
    pub_pcl_0_.publish(cloud_ros_down_sampling);
    pub_pcl_1_.publish(cloud_ros_plane_removal);
    pub_obstacles_pcl_clustering_.publish(cloud_obstacle_array_pre);

    // ros::Time end_time = ros::Time::now();
    // ROS_INFO("process took %f seconds", end_time.toSec() - start_time.toSec());

    return;
    
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