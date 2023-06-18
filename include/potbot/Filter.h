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

// Unscented Kalman Filter
class UKF{
	private:
		// 引数として受け取る関数の型
		typedef Eigen::VectorXd (*ModelFunction)(Eigen::VectorXd, double);
		ModelFunction f, h;
		Eigen::VectorXd xhat;
		Eigen::MatrixXd P,Q,R;

		// U変換(unscented transform)
		inline std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> U_transform(ModelFunction f_ut, Eigen::VectorXd xm, Eigen::MatrixXd Pxx, double dt) 
		{
			double n = xm.rows();
			double kappa = 3.0-n;

			Eigen::VectorXd w(2*(int)n+1);
			w.fill(1.0/(2*(n+kappa)));
			w(0) = kappa/(n+kappa);

			Eigen::MatrixXd L;
			Eigen::LLT<Eigen::MatrixXd> llt(Pxx);  // Pをコレスキー分解
			if (llt.info() == Eigen::Success) {
				L = llt.matrixL();  // 下三角行列Lを取得
			} else {
				std::cout << "Matrix is not positive definite." << std::endl;
			}

			//シグマポイントを作成
			Eigen::MatrixXd X((int)n, 2*(int)n+1);
			X.col(0)=xm;
			X.block(0, 1,					(X.cols()-1)/2, (X.cols()-1)/2) = xm*Eigen::RowVectorXd::Ones((int)n) + sqrt(n+kappa)*L;
			X.block(0, (X.cols()-1)/2+1,	(X.cols()-1)/2, (X.cols()-1)/2) = xm*Eigen::RowVectorXd::Ones((int)n) - sqrt(n+kappa)*L;

			//シグマポイントをfで変換
			int num_dim = f_ut(X.col(0),dt).rows();
			Eigen::MatrixXd Y(num_dim, X.cols());
			for(int i=0; i<Y.cols(); i++) Y.col(i) = f_ut(X.col(i),dt);

			// 重みをかけながら行ごとの総和を計算する(yの期待値計算)
			Eigen::VectorXd ym = (Y.array().rowwise() * w.transpose().array()).rowwise().sum();

			//ここが怪しい
			Eigen::MatrixXd Pyy(num_dim, num_dim);
			for(int i=0; i<Y.cols(); i++) Pyy += w(i)*(Y.col(i) - ym)*(Y.col(i) - ym).transpose();
			
			//ここが怪しい
			Eigen::MatrixXd Pxy(X.rows(), num_dim);
			for(int i=0; i<Y.cols(); i++) Pxy += w(i)*(X.col(i) - xm)*(Y.col(i) - ym).transpose();
			
			//std::cout<<Pyy<<std::endl;
			return std::make_tuple(ym, Pyy, Pxy);
		};

	public:
		UKF(ModelFunction model_func_system, ModelFunction model_func_observ, 
			Eigen::MatrixXd noise_covariance_system, Eigen::MatrixXd noise_covariance_observ, 
			Eigen::MatrixXd error_covariance_ini, Eigen::VectorXd estimate_state_ini)
		{
			f = model_func_system;
			h = model_func_observ;
			R = noise_covariance_system;
			Q = noise_covariance_observ;
			P = error_covariance_ini;
			xhat = estimate_state_ini;
		};
		~UKF(){};

		inline std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::VectorXd y, double dt)
		{
			std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans1 = U_transform(f,xhat,P,dt);

			Eigen::VectorXd xhatm = std::get<0>(ans1);
			Eigen::MatrixXd Pm = std::get<1>(ans1);

			Pm += R;

			std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans2 = U_transform(h,xhatm,Pm,dt);
			Eigen::VectorXd yhatm = std::get<0>(ans2);
			Eigen::MatrixXd Pyy = std::get<1>(ans2);
			Eigen::MatrixXd Pxy = std::get<2>(ans2);

			Eigen::MatrixXd G = Pxy*(Pyy+Q).inverse();			//カルマンゲイン
			Eigen::VectorXd xhat_new = xhatm + G*(y-yhatm);		//推定値
			Eigen::MatrixXd P_new = Pm - G*Pxy.transpose();		//推定誤差共分散

			// std::cout<<G<<std::endl;
			return std::make_tuple(xhat_new, P_new, G);
		}
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
		std::vector<UKF> states_ukf_;

        tf::TransformListener tflistener;
        tf2_ros::Buffer tf_buffer_;

        int robot_id_ = MEGAROVER;
        
        geometry_msgs::Twist cmd_;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        int robot_path_index_ = 0;
        nav_msgs::Odometry robot_, odom_;
        double SIGMA_P, SIGMA_Q, SIGMA_R;

        void __odom_callback(const nav_msgs::Odometry& msg);
        void __obstacle_callback(const visualization_msgs::MarkerArray& msg);

    public:
        FilterClass();
        ~FilterClass();
        void setLaunchParam();//launchファイルから書き込み
        
        void mainloop();

        void manage();
        void filter();
        
};
