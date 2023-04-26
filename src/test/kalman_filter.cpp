#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
//#include <beego_control/beego_encoder.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/Geometry> //EigenのGeometry関連の関数を使う場合，これが必要
#include <random>

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
		KalmanFilter()
		{
			An.resize(4,4);
			An<<1,0,dt,0,
				0,1,0,dt,
				0,0,1,0,
				0,0,0,1;

			xhat.resize(4,1);
			xtilde.resize(4,1);
			z.resize(4,1);

			Phat.resize(4,4);
			Phat(0,0) = Phat(1,1) = Phat(2,2) = Phat(3,3) = 10000;

			obse_sigma.resize(4,4);
			model_sigma.resize(4,4);
			obse_sigma(0,0) = obse_sigma(1,1) = obse_sigma(2,2) = obse_sigma(3,3) = 0.1;
			model_sigma(0,0) = model_sigma(1,1) = model_sigma(2,2) = model_sigma(3,3) = 0.01;

			K.resize(4,4);
			I = Eigen::MatrixXd::Identity(4,4);
			
		};

		void input_data(double data, double t_now)
		{
			z(2,0) = data;
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

		void update()
		{
			Ptilde = An*Phat*(An.transpose()) + model_sigma;

			K = Ptilde*((Ptilde+obse_sigma).inverse());
			xhat = xtilde + K*(z - xtilde);
			Phat = (I - K)*Ptilde;
			xtilde = An*xhat;
		};

		Eigen::MatrixXd get_state()
		{
			return xhat;
		};

};



int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_te");

	ros::NodeHandle nhSub,nhPub;
	ros::Publisher pub_odom;
	ros::Subscriber sub_encoder;
	
	KalmanFilter obs0;

	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		double t = ros::Time::now().toSec();
		double vx = sin(2.0*M_PI*0.05*t);
		obs0.input_data(vx,t);
		obs0.update();
		std::cout<<obs0.get_state().transpose()<<std::endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

