//include haeders
#include <potbot/Utility.h>
#include <potbot/State.h>
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
		ros::Subscriber sub_odom_, sub_obstacle_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_state_;

        visualization_msgs::MarkerArray obstacles_;
        std::vector<KalmanFilter> states_;

        tf::TransformListener tflistener;
        tf2_ros::Buffer tf_buffer_;

        int robot_id_ = MEGAROVER;
        
        geometry_msgs::Twist cmd_;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        int robot_path_index_ = 0;
        nav_msgs::Odometry robot_, odom_;
        std::string ROBOT_NAME;
        bool IS_SIMULATOR, PUBLISH_COMMAND;
        double PATH_TRACKING_MARGIN;

        void __odom_callback(const nav_msgs::Odometry& msg);
        void __obstacle_callback(const visualization_msgs::MarkerArray& msg);
        void __publish_path_request();
        void __publishcmd();

        void __LineFollowing();
        void __PoseAlignment();
        bool __PathCollision();

    public:
        FilterClass();
        ~FilterClass();
        void setLaunchParam();//launchファイルから書き込み
        
        void mainloop();

        void manage();
        void filter();
        
};
