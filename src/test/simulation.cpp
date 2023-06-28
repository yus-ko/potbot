#include <ros/ros.h>
#include<potbot/Filter.h>
#include <Eigen/Core>
#include <random>
#include <visualization_msgs/MarkerArray.h>

// #include <Eigen/Geometry>

#define __NX__ 5
#define __NY__ 2

Eigen::VectorXd u(double t)
{
	Eigen::VectorXd ans(2);
	ans<<	0.1*sin(2*M_PI*1/20*t),
			0.01;
	return ans;
}

Eigen::VectorXd f(Eigen::VectorXd x,Eigen::VectorXd u, double dt)
{
	Eigen::VectorXd ans(5);
	ans<<	x(0) + u(0)*cos(u(1))*dt,
			x(1) + u(0)*sin(u(1))*dt,
			x(2) + u(1)*dt,
			u(0),
			u(1);
	return ans;
}

Eigen::VectorXd f1(Eigen::VectorXd x_old, double dt) {
	Eigen::VectorXd x_new(__NX__);

	x_new(0) = x_old(0) + x_old(3)*cos(x_old(4)*dt)*dt;
	x_new(1) = x_old(1) + x_old(3)*sin(x_old(4)*dt)*dt;
	x_new(2) = x_old(2) + x_old(4)*dt;
	x_new(3) = x_old(3);
	x_new(4) = x_old(4);

	// x_new(0) = x_old(0) + x_old(2)*dt;
	// x_new(1) = x_old(1) + x_old(3)*dt;
	// x_new(2) = x_old(2);
	// x_new(3) = x_old(3);

	return x_new;
}

Eigen::VectorXd h(Eigen::VectorXd x, double dt) {
	Eigen::VectorXd y(__NY__);
	y(0) = x(0);
	y(1) = x(1);
	return y;
}

void matrixToDoubleArray(const Eigen::MatrixXd& matrix, std_msgs::Float64MultiArray& array) {
    int rows = matrix.rows();
    int cols = matrix.cols();

    int index = 0;
    for (int j = 0; j < cols; j++) {
        for (int i = 0; i < rows; i++) {
            array.data[index] = matrix(i, j);
            index++;
        }
    }
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_te");

	std::random_device rd;
    std::mt19937 gen(rd());  // メルセンヌ・ツイスター法を使用して乱数生成器を初期化

	ros::NodeHandle nhPub;
	ros::Publisher pub_robot = nhPub.advertise<nav_msgs::Odometry>("simulation/robot/odom", 1);
	ros::Publisher pub_seg = nhPub.advertise<visualization_msgs::MarkerArray>("/robot_0/segment_test", 1);
	ros::Publisher pub_state = nhPub.advertise<potbot::State>("/robot_0/state", 1);

	ros::Rate loop_rate(50);
	
	ros::WallTime time_begin = ros::WallTime::now();

	Eigen::VectorXd x(5),y(5),E(5),F(5);
	x.setZero();E.setZero();F.setOnes();

	Eigen::MatrixXd A(5,5),B(2,5),C(5,5);
	C<< 1, 0, 0, 0, 0,
		0, 1, 0, 0, 0,
		0, 0, 1, 0, 0,
		0, 0, 0, 1, 0,
		0, 0, 0, 0, 1;

	double mean = 0.0;  // 平均値
    double stddev = 0.01;  // 標準偏差

    std::normal_distribution<double> dist_x(0.0, 1e-9), dist_y(0.0, 1e-3), dist_y1(0.0, 1e-2);  // 正規分布乱数生成器を作成

	Eigen::MatrixXd Q(__NY__,__NY__), R(__NX__,__NX__), P(__NX__,__NX__);
	Q.setZero();R.setZero();P.setZero();
	for (int i = 3; i < __NX__; i++) R(i,i) = 1e-5;
	for (int i = 0; i < __NY__; i++) Q(i,i) = 1e-2;
	for (int i = 0; i < __NX__; i++) P(i,i) = 1e2;

	Eigen::VectorXd xhat(__NX__);
	xhat.setZero();
	
	UKF estimate(f1,h,R,Q,P,xhat);

	ROS_INFO("begin");
	while (ros::ok())
	{
		nav_msgs::Odometry robot;
		ros::Time t_now = ros::Time::now();
		robot.header.stamp = t_now;
    	robot.header.frame_id = "/map";

		static ros::Time t_pre = t_now;
		
		double dt = t_now.toSec() - t_pre.toSec();
		t_pre = t_now;
		if(dt>1) continue;

		double t = t_now.toSec() - time_begin.toSec();
		double v = dist_x(gen);
		double w = dist_y(gen);
		x = f(x,u(t),dt) + v*E;
		y = C*x + w*F;

		robot.pose.pose.position.x = y(0);
		robot.pose.pose.position.y = y(1);
		
		geometry_msgs::Quaternion quat;
		getQuat(0,0,y(2),quat);
		robot.pose.pose.orientation = quat;

		robot.twist.twist.linear.x = y(3);
		robot.twist.twist.angular.z = y(4);

		// std::cout<<t_now.toSec()<<std::endl;
		//ROS_INFO("x,y= %f, %f", y(0),y(1));

		visualization_msgs::MarkerArray seg;
		int id = 0;
		for (int i = 0; i < 1; i++)
		{

			visualization_msgs::Marker segment;
			segment.header = robot.header;
			segment.header.frame_id = "lidar";

			segment.ns = "segments_display";
			segment.id = i;
			segment.lifetime = ros::Duration(1);

			segment.type = visualization_msgs::Marker::SPHERE;
			segment.action = visualization_msgs::Marker::MODIFY;

			
			segment.pose.position.x = x(0) + dist_y1(gen);
			segment.pose.position.y = x(1) + dist_y1(gen);
			segment.pose.position.z = 0;

			segment.pose.orientation.x = 0;
			segment.pose.orientation.y = 0;
			segment.pose.orientation.z = 0;
			segment.pose.orientation.w = 1;

			segment.scale.x = 0.1;
			segment.scale.y = 0.1;

			segment.scale.z = 0.001;

			segment.color.a = 0.3;

			segment.color.r = 0.9;
			segment.color.g = 0;
			segment.color.b = 0;
			
			seg.markers.push_back(segment);
		}
		
		Eigen::VectorXd observed_data(__NY__);
		observed_data<< y(0), y(1);
		std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = estimate.update(observed_data,dt);
		Eigen::MatrixXd xhat = std::get<0>(ans);
		Eigen::MatrixXd P = std::get<1>(ans);
		Eigen::MatrixXd K = std::get<2>(ans);

		potbot::State state_msg;
		state_msg.header = robot.header;

		state_msg.z.data.resize(__NY__);
		state_msg.xhat.data.resize(xhat.rows());
		state_msg.P.data.resize(P.rows()*P.cols());
		state_msg.K.data.resize(K.rows()*K.cols());

		matrixToDoubleArray(observed_data, state_msg.z);
		matrixToDoubleArray(xhat, state_msg.xhat);
		matrixToDoubleArray(P, state_msg.P);
		matrixToDoubleArray(K, state_msg.K);
		
		std::cout<<xhat(3)<<std::endl;
		pub_state.publish(state_msg);

		pub_robot.publish(robot);
		pub_seg.publish(seg);
		
		//ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("finish");

	return 0;
}

