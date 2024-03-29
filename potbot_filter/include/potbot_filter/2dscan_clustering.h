//include haeders
#include <potbot_lib/Utility.h>
#include <potbot_lib/ScanClustering.h>
#include <potbot_msgs/State.h>
#include <potbot_msgs/StateArray.h>
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

//クラスの定義
class scan2dClass{

    private:
        
        //センサーデータ
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_scan_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_scan_filter_, pub_segment_, pub_obstacles_scan_clustering_;

        visualization_msgs::MarkerArray obstacles_;

        //tf::TransformListener tflistener;
        tf2_ros::Buffer tf_buffer_;
        
		sensor_msgs::LaserScan scan_;
        std::vector<sensor_msgs::LaserScan> scans_;
        int Tn_=30;
        double square_width_=0.1;
		
		dynamic_reconfigure::Server<potbot_msgs::ClusteringConfig> server_;
  	    dynamic_reconfigure::Server<potbot_msgs::ClusteringConfig>::CallbackType f_;

		std::string FRAME_ID_GLOBAL = "map";
		std::string FRAME_ID_ROBOT_BASE = "base_link";
		std::string TOPIC_SCAN = "scan";

		void __param_callback(const potbot_msgs::ClusteringConfig& param, uint32_t level);
		void __scan_callback(const sensor_msgs::LaserScan::ConstPtr msg);

		void __get_param();

		double __Median(std::vector<double> v);
        void __MedianFilter(sensor_msgs::LaserScan &scan);
        void __Segmentation(sensor_msgs::LaserScan &scan, std::vector<SEGMENT> &segments);
        double __distanceToLineSegment(POINT o, POINT p, POINT q);
        void __SplitSegments(std::vector<SEGMENT> &segments);
        void __AssociateSegments(std::vector<SEGMENT> &segments);

    public:
        scan2dClass();
        ~scan2dClass(){};
        
        void mainloop();
        
};
