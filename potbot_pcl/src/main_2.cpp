#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <iomanip> // for setw, setfill

#include <pcl/segmentation/supervoxel_clustering.h>

#include <visualization_msgs/MarkerArray.h>

class EuclideanClustering{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
    ros::Publisher pub_pc;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*pcl objects*/
		pcl::visualization::PCLVisualizer viewer {"Euclidian Clustering"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw {new pcl::PointCloud<pcl::PointXYZ>}, cloud_f {new pcl::PointCloud<pcl::PointXYZ>}, cloud_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		/*parameters*/
		double cluster_tolerance;
		int min_cluster_size;
	public:
		EuclideanClustering();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Clustering(void);
		void Visualization(void);
};

EuclideanClustering::EuclideanClustering()
	:nhPrivate("~")
{
  pub_pc = nh.advertise<visualization_msgs::MarkerArray>("cluster_pcl", 1);
	sub_pc = nh.subscribe("/robot_4/realsense/depth/points", 1, &EuclideanClustering::CallbackPC, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);

	nhPrivate.param("cluster_tolerance", cluster_tolerance, 0.5);
	nhPrivate.param("min_cluster_size", min_cluster_size, 100);
	std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
	std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
}

void EuclideanClustering::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */
	pcl::fromROSMsg(*msg, *cloud_raw);
	std::cout << "==========" << std::endl;
	std::cout << "cloud_raw->points.size() = " << cloud_raw->points.size() << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud_raw);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl;

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }



  float voxel_resolution = 0.01f;

  float seed_resolution = 0.3f;

  float color_importance = 0.2f;

  float spatial_importance = 0.4f;

  float normal_importance = 1.0f;

  pcl::SupervoxelClustering<pcl::PointXYZ> super (voxel_resolution, seed_resolution);
  super.setInputCloud (cloud_filtered);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <std::uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
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
    for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<pcl::PointXYZ>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      int a;
      a=1;
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }

    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    std_msgs::ColorRGBA random_color;
    random_color.r = dis(gen);
    random_color.g = dis(gen);
    random_color.b = dis(gen);
    random_color.a = 1;

    for(int i = 0; i < adjacent_supervoxel_centers.size(); i++)
    {
      visualization_msgs::Marker marker;

      std::cout << adjacent_supervoxel_centers[i] << std::endl;
      float x = adjacent_supervoxel_centers[i].x;
      float y = adjacent_supervoxel_centers[i].y;
      float z = adjacent_supervoxel_centers[i].z;

      // marker.header.frame_id = "robot_4/stereo_camera_optical_frame";
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "cluster_display";
      marker.id = marker_array.markers.size();
      marker.lifetime = ros::Duration();

      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color = random_color;
      

      marker_array.markers.push_back(marker);
      
    }
    

    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    // addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }
  pub_pc.publish(marker_array);

  // return;

  // // Creating the KdTree object for the search method of the extraction
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // tree->setInputCloud (cloud_filtered);

  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // ec.setClusterTolerance (0.02); // 2cm
  // ec.setMinClusterSize (100);
  // ec.setMaxClusterSize (25000);
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (cloud_filtered);
  // ec.extract (cluster_indices);

  // int j = 0;
  // for (const auto& cluster : cluster_indices)
  // {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  //   for (const auto& idx : cluster.indices) {
  //     cloud_cluster->push_back((*cloud_filtered)[idx]);
  //   } //*
  //   cloud_cluster->width = cloud_cluster->size ();
  //   cloud_cluster->height = 1;
  //   cloud_cluster->is_dense = true;

  //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
  //   std::stringstream ss;
  //   ss << std::setw(4) << std::setfill('0') << j;
  //   writer.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
  //   j++;
  // }

  // return;




  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*cloud_raw, *cloud, indices);
  // std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;
  // // for(int i = 0; i < cloud_raw->points.size(); i++)
  // // {
  // //   double x = cloud_raw->points[i].x;
  // //   double y = cloud_raw->points[i].y;
  // //   double z = cloud_raw->points[i].z;
  // //   if (!isnan(x) && !isnan(y) && !isnan(z))
  // //   {
  // //     ROS_INFO("%f, %f, %f", x,y,z);
  // //   }

  // // }

  // // KD木を作っておく。近傍点探索とかが早くなる。
  // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  // tree->setInputCloud(cloud);

  // //近傍点探索に使うパラメータと結果が入る変数
  // double radius = 5;  //半径r
  // std::vector<int> k_indices;  //範囲内の点のインデックスが入る
  // std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
  // unsigned int max_nn = 1;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
  
  // pcl::PointXYZ p;  //中心座標
  // p.x = 0;
  // p.y = 0;
  // p.z = 1;

  // //半径r以内にある点を探索
  // tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
  
  // if(k_indices.size() == 0) return;
  
  // pcl::PointXYZ result = cloud->points[k_indices[0]];

  // ROS_INFO("A nearest point of (%f, %f, %f) is...\nx: %lf, y:%lf, z:%lf",p.x,p.y,p.z, result.x, result.y, result.z);
  // return;



	clusters.clear();
	Clustering();
	Visualization();
}

void EuclideanClustering::Clustering(void)
{
	double time_start = ros::Time::now().toSec();

	/*clustering*/
	/*kd-treeクラスを宣言*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	/*探索する点群をinput*/
	tree->setInputCloud(cloud_filtered);
	/*クラスタリング後のインデックスが格納されるベクトル*/
	std::vector<pcl::PointIndices> cluster_indices;
	/*今回の主役*/
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	/*距離の閾値を設定*/
	ece.setClusterTolerance(cluster_tolerance);
	/*各クラスタのメンバの最小数を設定*/
	ece.setMinClusterSize(min_cluster_size);
	/*各クラスタのメンバの最大数を設定*/
	ece.setMaxClusterSize(cloud_filtered->points.size());
	/*探索方法を設定*/
	ece.setSearchMethod(tree);
	/*クラスリング対象の点群をinput*/
	ece.setInputCloud(cloud_filtered);
	/*クラスリング実行*/
	ece.extract(cluster_indices);

	std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	/*dividing（クラスタごとに点群を分割）*/
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud_filtered);
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

	std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void EuclideanClustering::Visualization(void)
{
	/*前ステップの可視化をリセット*/
	viewer.removeAllPointClouds();

	/*cloud*/
	viewer.addPointCloud(cloud_filtered, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	/*clusters*/
	double rgb[3] = {};
	const int channel = 3;	//RGB
	const double step = ceil(pow(clusters.size()+2, 1.0/(double)channel));	//exept (000),(111)
	const double max = 1.0;
	/*クラスタをいい感じに色分け*/
	for(size_t i=0;i<clusters.size();i++){
		std::string name = "cluster_" + std::to_string(i);
		rgb[0] += 1/step;
		for(int j=0;j<channel-1;j++){
			if(rgb[j]>max){
				rgb[j] -= max + 1/step;
				rgb[j+1] += 1/step;
			}
		}
		viewer.addPointCloud(clusters[i], name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
	}
	/*表示の更新*/
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "euclidean_clustering");
	
	EuclideanClustering euclidean_clustering;

	ros::spin();
}