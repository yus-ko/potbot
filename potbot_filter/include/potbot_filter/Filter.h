//include haeders
#include <potbot_lib/Utility.h>
#include <potbot_lib/UnscentedKalmanFilter.h>
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
#include <potbot_filter/FilterConfig.h>

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
               } SEGMENT;

class KalmanFilter{
	private:
		Eigen::MatrixXd Phat, Ptilde, xtilde,
		xhat,			//推定状態
		z,				//観測データ
		An,				//障害物の移動モデル
		obse_sigma,		//観測誤差共分散
		model_sigma,	//モデル化誤差共分散
		K,				//カルマンゲイン
		I;				//単位行列

		double dt = -1;		//サンプリング時間

	public:
		inline KalmanFilter()
		{
			An.resize(4,4);
			An<<1,0,dt,0,
				0,1,0,dt,
				0,0,1,0,
				0,0,0,1;

			xhat.resize(4,1);
			xtilde.resize(4,1);
			z.resize(4,1);
			xhat<<	0,
					0,
					0,
					0;
			xtilde = z = xhat;

			Phat.resize(4,4);
			Phat<<	10000,0,0,0,
					0,10000,0,0,
					0,0,10000,0,
					0,0,0,10000;

			obse_sigma.resize(4,4);
			model_sigma.resize(4,4);
			obse_sigma<<	0.01,0,0,0,
							0,0.01,0,0,
							0,0,0.01,0,
							0,0,0,0.01;
			model_sigma = obse_sigma;

			K.resize(4,4);
			K<<	0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0;
			I = Eigen::MatrixXd::Identity(4,4);
			
		};

		inline void input_data(Eigen::MatrixXd data, double t_now)
		{
			z = data;
			static double t_pre;
			if (dt >= 0)
			{
				dt = t_now - t_pre;
			}
			else
			{
				dt = 0;
			}
			t_pre = t_now;
			An(0,2) = dt;
			An(1,3) = dt;
		};

		inline void update()
		{
			// std::cout<<I<<std::endl;
			Ptilde = An*Phat*An.transpose() + model_sigma;
			
			K = Ptilde*(Ptilde+obse_sigma).inverse();
			xhat = xtilde + K*(z - xtilde);
			Phat = (I - K)*Ptilde;

			xtilde = An*xhat;
		};

		inline Eigen::MatrixXd get_xhat()
		{
			return xhat;
		};

		inline Eigen::MatrixXd get_z()
		{
			return z;
		};

		inline Eigen::MatrixXd get_K()
		{
			return K;
		};

		inline Eigen::MatrixXd get_P()
		{
			return Phat;
		};

};

//クラスの定義
class FilterClass{

    private:
        
        //センサーデータ
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_obstacle_, sub_scan_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_state_, pub_scan0_, pub_scan1_, pub_segment_, pub_state_markers_, pub_obstacles_scan_, pub_obstacles_pcl_, pub_obstacles_scan_test_;

        visualization_msgs::MarkerArray obstacles_;
        std::vector<KalmanFilter> states_;
		std::vector<potbot_lib::UnscentedKalmanFilter> states_ukf_;

        //tf::TransformListener tflistener;
        tf2_ros::Buffer tf_buffer_;
        
		sensor_msgs::LaserScan scan_;
        std::vector<sensor_msgs::LaserScan> scans_;
        int Tn_=30;
        double square_width_=0.1;

        geometry_msgs::Twist cmd_;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        int robot_path_index_ = 0;
        nav_msgs::Odometry robot_;
		
		dynamic_reconfigure::Server<potbot_filter::FilterConfig> server_;
  	    dynamic_reconfigure::Server<potbot_filter::FilterConfig>::CallbackType f_;

		std::string FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, TOPIC_SCAN;
        double SIGMA_P, SIGMA_Q, SIGMA_R;

        // void __obstacle_callback(const visualization_msgs::MarkerArray& msg);
		void __obstacle_callback(const potbot_msgs::ObstacleArray& msg);
		void __scan_callback(const sensor_msgs::LaserScan& msg);
		
		void __param_callback(const potbot_filter::FilterConfig& param, uint32_t level);

		double __Median(std::vector<double> v);
        void __MedianFilter(sensor_msgs::LaserScan &scan);
        void __Segmentation(sensor_msgs::LaserScan &scan, std::vector<SEGMENT> &segments);
        double __distanceToLineSegment(POINT o, POINT p, POINT q);
        void __SplitSegments(std::vector<SEGMENT> &segments);
        void __AssociateSegments(std::vector<SEGMENT> &segments);

    public:
        FilterClass();
        ~FilterClass();
        void setLaunchParam();//launchファイルから書き込み
        
        void mainloop();

        void manage();
        void filter();
        
};
