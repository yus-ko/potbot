#ifndef _H_2DSCAN_CLUSTERING_
#define _H_2DSCAN_CLUSTERING_

#include <potbot_lib/Utility.h>
#include <potbot_lib/ScanClustering.h>
#include <potbot_msgs/ObstacleArray.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_msgs/ClusteringConfig.h>

#define NO_SEGMENT -1

typedef struct {
                int index=0;
                double x=0;
                double y=0;
                double r=0;
                double theta=0;
               } POINT;

typedef struct {
                std::vector<POINT> points;
                int id=0;
                int type=0;
                double x=0;
                double y=0;
                double radius=0;
                double width=0;
                double height=0;
				bool is_moving=false;
               } SEGMENT;

class scan2dClass{

    private:
        
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_scan_;
        
        ros::NodeHandle nhPub_;
		ros::Publisher pub_scan_filter_, pub_segment_, pub_obstacles_scan_clustering_;

        visualization_msgs::MarkerArray obstacles_;

        tf2_ros::Buffer& tf_buffer_;
        
		sensor_msgs::LaserScan scan_;
        std::vector<sensor_msgs::LaserScan> scans_;
        int Tn_=30;
        double square_width_=0.1;
		
        dynamic_reconfigure::Server<potbot_msgs::ClusteringConfig> *dsrv_;

		std::string frame_id_global_ = "map";
		std::string frame_id_robot_base_ = "base_link";
		std::string topic_scan_ = "scan";

		void __param_callback(const potbot_msgs::ClusteringConfig& param, uint32_t level);
		void __scan_callback(const sensor_msgs::LaserScan::ConstPtr msg);

		double __Median(std::vector<double> v);
        void __MedianFilter(sensor_msgs::LaserScan &scan);
        void __Segmentation(sensor_msgs::LaserScan &scan, std::vector<SEGMENT> &segments);
        double __distanceToLineSegment(POINT o, POINT p, POINT q);
        void __SplitSegments(std::vector<SEGMENT> &segments);
        void __AssociateSegments(std::vector<SEGMENT> &segments);

    public:
        scan2dClass(tf2_ros::Buffer& tf, const std::string& name = "");
        ~scan2dClass(){};
        
};
#endif // _H_2DSCAN_CLUSTERING_
