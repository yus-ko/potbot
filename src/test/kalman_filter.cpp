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
		Eigen::MatrixXd m1,m2,result,Phat, Ptilde, An, model_sigma, obse_sigma, K, xhat, xtilde, z;

	public:
		KalmanFilter(Eigen::MatrixXd ini1)
		{
			
			m2.resize(2,2);
			result.resize(2,2);

			// 行列に値を代入
			m1 = ini1;
				
			
			m2 << 5, 6,
				7, 8;

			
		};

		void add()
		{
			result = m1 + m2;
		};

		void show()
		{
			std::cout << result << std::endl;
		};

		void update()
		{
			Ptilde = An*Phat*An.transpose() + model_sigma;

			K = Ptilde*(Ptilde+obse_sigma).inverse();
			xhat = xtilde + K*(z - xtilde);
			Eigen::MatrixXd I = Eigen::MatrixXd::Identity(K.rows(), K.cols());
			Phat = (I - K)*Ptilde;
		};

};



int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_te");

	ros::NodeHandle nhSub,nhPub;
	ros::Publisher pub_odom;
	ros::Subscriber sub_encoder;

	Eigen::MatrixXd mat(2,2);
	mat << 1, 1,
			1, 1;
	KalmanFilter a(mat);
	a.add();
	a.show();

	return 0;
}

