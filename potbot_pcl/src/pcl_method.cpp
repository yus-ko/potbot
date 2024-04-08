#include<potbot_pcl/Clustering3D.h>

void Clustering3DClass::__DownSampling()
{
    //データセットのダウンサンプリング
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_raw_);
    vg.setLeafSize(DownSampling_voxel_size_, DownSampling_voxel_size_, DownSampling_voxel_size_);
    vg.filter (*cloud_filtered_);
    // ROS_INFO("PointCloud after filtering has: %d data points.", cloud_filtered_->size());

    pcl::PCLPointCloud2 cloud_pointcloud2;
    pcl::toPCLPointCloud2(*cloud_filtered_, cloud_pointcloud2);
    sensor_msgs::PointCloud2 cloud_ros;
    pcl_conversions::fromPCL(cloud_pointcloud2, cloud_ros);
    pub_pcl_0_.publish(cloud_ros);
}

void Clustering3DClass::__Plane_removal()
{
    // 平面モデル作成のためのパラメーター
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (PlaneRemoval_distance_threshold_);

    int nr_points = (int) cloud_filtered_->size ();
    while (cloud_filtered_->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered_);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            // ROS_ERROR("Could not estimate a planar model for the given dataset.");
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered_);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        // ROS_INFO("PointCloud representing the planar component: %d data points.", cloud_plane->size());

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        //cloud_fをメンバ変数からローカル変数に変更
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f {new pcl::PointCloud<pcl::PointXYZ>};
        extract.filter (*cloud_f);
        *cloud_filtered_ = *cloud_f;
    }
    pcl::PCLPointCloud2 cloud_pointcloud2;
    pcl::toPCLPointCloud2(*cloud_filtered_, cloud_pointcloud2);
    sensor_msgs::PointCloud2 cloud_ros;
    pcl_conversions::fromPCL(cloud_pointcloud2, cloud_ros);
    pub_pcl_1_.publish(cloud_ros);
}

void Clustering3DClass::__SuperVoxelClustering()
{
    pcl::SupervoxelClustering<pcl::PointXYZ> super (Supervoxel_voxel_resolution_, Supervoxel_seed_resolution_);
    super.setInputCloud (cloud_filtered_);
    super.setColorImportance (Supervoxel_color_importance_);
    super.setSpatialImportance (Supervoxel_spatial_importance_);
    super.setNormalImportance (Supervoxel_normal_importance_);

    std::map <std::uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr > supervoxel_clusters;

    // ROS_INFO("Extracting supervoxels!");
    super.extract (supervoxel_clusters);
    // ROS_INFO("Found %d supervoxels", supervoxel_clusters.size());

    // ROS_INFO("Getting supervoxel adjacency");
    std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);

    visualization_msgs::MarkerArray marker_array;
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
    {
        //First get the label
        std::uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZ>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        pcl::PointCloud<pcl::PointXYZ> adjacent_supervoxel_voxels;
        for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
            pcl::Supervoxel<pcl::PointXYZ>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
            adjacent_supervoxel_voxels = *(neighbor_supervoxel->voxels_);
        }

        visualization_msgs::Marker marker_centor;
        marker_centor.header = header_;
        marker_centor.ns = "SuperVoxel/centor";
        marker_centor.lifetime = ros::Duration(0.2);
        marker_centor.type = visualization_msgs::Marker::CUBE;
        marker_centor.action = visualization_msgs::Marker::ADD;
        marker_centor.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (adjacent_supervoxel_voxels, minPt, maxPt);
        if(abs(maxPt.x - minPt.x) == 0) marker_centor.scale.x = Supervoxel_voxel_resolution_;
        else                            marker_centor.scale.x = abs(maxPt.x - minPt.x);
        if(abs(maxPt.y - minPt.y) == 0) marker_centor.scale.y = Supervoxel_voxel_resolution_;
        else                            marker_centor.scale.y = abs(maxPt.y - minPt.y);  
        if(abs(maxPt.z - minPt.z) == 0) marker_centor.scale.x = Supervoxel_voxel_resolution_;
        else                            marker_centor.scale.z = abs(maxPt.z - minPt.z);
        
        for(int i = 0; i < adjacent_supervoxel_centers.size(); i++)
        {
            marker_centor.id = marker_array.markers.size();
            marker_centor.pose.position.x = adjacent_supervoxel_centers[i].x;
            marker_centor.pose.position.y = adjacent_supervoxel_centers[i].y;
            marker_centor.pose.position.z = adjacent_supervoxel_centers[i].z;
            marker_centor.color = potbot_lib::color::get_msg(marker_centor.id);
            marker_array.markers.push_back(marker_centor);

            if(i == 0)
            {
                visualization_msgs::Marker marker_voxels = marker_centor;
                marker_voxels.ns = "SuperVoxel/points";
                marker_voxels.id = marker_array.markers.size();

                marker_voxels.type = visualization_msgs::Marker::CUBE_LIST;

                marker_voxels.pose.position.x = 0;
                marker_voxels.pose.position.y = 0;
                marker_voxels.pose.position.z = 0;

                marker_voxels.scale.x = Supervoxel_voxel_resolution_;
                marker_voxels.scale.y = Supervoxel_voxel_resolution_;
                marker_voxels.scale.z = Supervoxel_voxel_resolution_;
                
                for(int i = 0; i < adjacent_supervoxel_voxels.size(); i++)
                {
                    geometry_msgs::Point point;
                    point.x = adjacent_supervoxel_voxels[i].x;
                    point.y = adjacent_supervoxel_voxels[i].y;
                    point.z = adjacent_supervoxel_voxels[i].z;
                    
                    marker_voxels.points.push_back(point);
                    marker_voxels.colors.push_back(marker_voxels.color);
                }
                marker_array.markers.push_back(marker_voxels);
            }
        }
        
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }
    pub_marker_.publish(marker_array);
}

void Clustering3DClass::__EuclideanClustering()
{
	double time_start = ros::Time::now().toSec();

	/*clustering*/
	/*kd-treeクラスを宣言*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	/*探索する点群をinput*/
	tree->setInputCloud(cloud_filtered_);
	/*クラスタリング後のインデックスが格納されるベクトル*/
	std::vector<pcl::PointIndices> cluster_indices;
	/*クラスタリング手法*/
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	/*距離の閾値を設定*/
	ece.setClusterTolerance(Euclidean_cluster_tolerance_);
	/*各クラスタのメンバの最小数を設定*/
	ece.setMinClusterSize(Euclidean_min_cluster_size_);
	/*各クラスタのメンバの最大数を設定*/
	ece.setMaxClusterSize(cloud_filtered_->points.size());
	/*探索方法を設定*/
	ece.setSearchMethod(tree);
	/*クラスリング対象の点群をinput*/
	ece.setInputCloud(cloud_filtered_);
	/*クラスリング実行*/
	ece.extract(cluster_indices);

    // ROS_INFO("cluster_indices.size() = %d", cluster_indices.size());

	/*dividing（クラスタごとに点群を分割）*/
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud_filtered_);
	ei.setNegative(false);
	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		ei.setIndices(tmp_clustered_indices);
		ei.filter(*tmp_clustered_points);
		/*input*/
		clusters.push_back(tmp_clustered_points);
	}

    visualization_msgs::MarkerArray marker_array;
    for(size_t i=0;i<clusters.size();i++)
    {
        visualization_msgs::Marker marker_centor;

        marker_centor.header = header_;
        marker_centor.ns = "Euclidean/centor";
        marker_centor.id = marker_array.markers.size();
        marker_centor.lifetime = ros::Duration(0.2);

        marker_centor.type = visualization_msgs::Marker::SPHERE;
        marker_centor.action = visualization_msgs::Marker::ADD;
        
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*(clusters[i]), minPt, maxPt);
        marker_centor.pose.position.x = (maxPt.x - minPt.x)/2 + minPt.x;
        marker_centor.pose.position.y = (maxPt.y - minPt.y)/2 + minPt.y;
        marker_centor.pose.position.z = (maxPt.z - minPt.z)/2 + minPt.z;

        marker_centor.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);

        marker_centor.scale.x = abs(maxPt.x - minPt.x);
        marker_centor.scale.y = abs(maxPt.y - minPt.y);
        marker_centor.scale.z = abs(maxPt.z - minPt.z);

        marker_centor.color = potbot_lib::color::get_msg(marker_centor.id);
        marker_array.markers.push_back(marker_centor);

        visualization_msgs::Marker marker_points = marker_centor;
        marker_points.ns = "Euclidean/points";
        marker_points.id = marker_array.markers.size();
        marker_points.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_points.pose.position.x = 0;
        marker_points.pose.position.y = 0;
        marker_points.pose.position.z = 0;
        marker_points.scale.x = DownSampling_voxel_size_;
        marker_points.scale.y = DownSampling_voxel_size_;
        marker_points.scale.z = DownSampling_voxel_size_;
        
        for(int j = 0; j < clusters[i]->points.size(); j++)
        {

            geometry_msgs::Point point;
            point.x = clusters[i]->points[j].x;
            point.y = clusters[i]->points[j].y;
            point.z = clusters[i]->points[j].z;
            
            marker_points.points.push_back(point);
            marker_points.colors.push_back(marker_centor.color);
        
        }
        marker_array.markers.push_back(marker_points);



    }
    if(!marker_array.markers.empty()) pub_marker_.publish(marker_array);

    // ROS_INFO("clustering time [s] = %f", ros::Time::now().toSec() - time_start);
}