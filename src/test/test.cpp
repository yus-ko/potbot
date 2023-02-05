#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
//#include <beego_control/beego_encoder.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/Geometry> //EigenのGeometry関連の関数を使う場合，これが必要
#include <random>

nav_msgs::Odometry odom;
bool first = true;
ros::Time encoder_time_now, encoder_time_pre;

// void encoder_callback(const beego_control::beego_encoder& msg)
// {
// 	ros::Time encoder_time_now = ros::Time::now();
// 	if(first)
// 	{
// 		first = false;
// 	}
// 	else
// 	{
// 		double ang = (-msg.vel.r - msg.vel.l) / 0.285;
// 		double vel = (-msg.vel.r + msg.vel.l) / 2.0;
// 		double encoder_deltatime = encoder_time_now.toSec() - encoder_time_pre.toSec();
// 		odom.pose.pose.orientation.z += ang * encoder_deltatime;
// 		odom.pose.pose.position.x += vel * cos(odom.pose.pose.orientation.z) * encoder_deltatime;
// 		odom.pose.pose.position.y += vel * sin(odom.pose.pose.orientation.z) * encoder_deltatime;

// 		//std::cout<< ros::WallTime::now() - start_time <<std::endl;
// 		std::cout<< msg <<std::endl;
// 		std::cout<< odom.pose.pose<<std::endl;
// 	}
// 	encoder_time_pre = encoder_time_now;
	
// 	//ROS_INFO("subscribe: %d", msg.data);
// }

void spline(std::vector<geometry_msgs::Vector3>& points)
{
	std::vector<geometry_msgs::Vector3> points_original = points;
    int N = points_original.size() - 1;
	Eigen::VectorXd x(N+1);
	Eigen::VectorXd y(N+1);

	for(int j=0; j<N+1; j++)
	{
		x[j] = points_original[j].x;
		y[j] = points_original[j].y;
	}

	Eigen::VectorXd h(N);
	for(int j=0; j<N; j++)
	{
		h[j] = x[j+1] - x[j];
	}

	Eigen::VectorXd v(N-1);
	for(int j=1; j<N; j++)
	{
		v[j-1] = 6*(((y[j+1] - y[j])/h[j]) - ((y[j] - y[j-1])/h[j-1]));
	}

	Eigen::MatrixXd H(N-1, N-1);
	for(int j=0; j<N-1; j++)
	{
		for(int k=0; k<N-1; k++)
		{
			if(j == 0)
			{
				if(k == 0)
				{
					H(j,k) = 2*(h[j] + h[j+1]);
				}
				else if (k == 1)
				{
					H(j,k) = h[j+1];
				}
				else
				{
					H(j,k) = 0.0;
				}
			}
			else if(j == N-2)
			{
				if(k == N-2)
				{
					H(j,k) = 2*(h[j] + h[j+1]);
				}
				else if (k == N-3)
				{
					H(j,k) = h[j];
				}
				else
				{
					H(j,k) = 0.0;
				}
			}
			else
			{
				if(k == j-1)
				{
					H(j,k) = h[j];
				}
				else if (k == j)
				{
					H(j,k) = 2*(h[j] + h[j+1]);
				}
				else if (k == j+1)
				{
					H(j,k) = h[j+1];
				}
				else
				{
					H(j,k) = 0.0;
				}
			}
		}
	}

	Eigen::FullPivLU<Eigen::MatrixXd> LU(H);
	Eigen::VectorXd U = LU.solve(v);
	
	int U_size = U.size();
	Eigen::VectorXd u(U_size+2);
	int idx = 0;
	for (int i = 0; i < U_size+2; i++)
	{
		if(i == 0 || i == U_size+1)
		{
			u(i) = 0.0;
		}
		else
		{
			u(i) = U(idx++);
		}
	}

	Eigen::VectorXd a(N);
	Eigen::VectorXd b(N);
	Eigen::VectorXd c(N);
	Eigen::VectorXd d(N);
	for (int j = 0; j < N; j++)
	{
		a(j) = (u(j+1) - u(j)) / (6*(x(j+1) - x(j)));
		b(j) = u(j) / 2;
		c(j) = ((y(j+1) - y(j)) / (x(j+1) - x(j))) - ((1.0/6.0)*(x(j+1) - x(j))*(2.0*u(j) + u(j+1)));
		d(j) = y(j);
	}

	points.resize(0);
	for (int j = 0; j < N; j++)
	{
		for (double t = x(j); t <= x(j+1); t+=0.1)
		{
			geometry_msgs::Vector3 ans;
			ans.x = t;
			// if (j == 0 && t == x(j))
			// {
			// 	ans.y = ;
			// }
			// else if ()
			// {

			// }
			// else
			// {

			// }
			double x_xj = t - x(j);
			ans.y = a(j)*pow(x_xj,3) + b(j)*pow(x_xj,2) + c(j)*x_xj + d(j);
			points.push_back(ans);
		}
	}

}

double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_te");

	ros::NodeHandle nhSub,nhPub;
	ros::Publisher pub_odom;
	ros::Subscriber sub_encoder;

    //sub_encoder = nhSub.subscribe("/encoder",1,encoder_callback);
	pub_odom = nhPub.advertise<nav_msgs::Odometry>("zed_node/odom", 1);

	//std::cout<< atan(6/0.0000001) /M_PI*180 <<std::endl;

	// std::vector<float> vec(5);
	// vec[0] = 4.0;
	// vec[1] = 4.0;
	// vec[2] = 4.0;
	// vec[3] = 1.0;
	// vec[4] = 4.0;

	// std::vector<float>::iterator minIt = std::min_element(vec.begin(), vec.end());
    // size_t minIndex = std::distance(vec.begin(), minIt);

	// //std::cout<< minIndex <<std::endl;
	// //std::cout<< *minIt <<std::endl;
	

	// float inf = std::numeric_limits<float>::infinity();
	// //std::cout<< inf+1 <<std::endl;

	// double tate = -1.0;
	// double yoko = 1.0;
	// double at = atan(tate/yoko);

	// if (yoko < 0 && tate > 0) 
	// {
	// 	at += M_PI;
	// }
	// else if (yoko < 0 && tate < 0) 
	// {
	// 	at += M_PI;
	// }
	// else if (yoko > 0 && tate < 0) 
	// {
	// 	at += 2*M_PI;
	// }

	// std::cout<< at/M_PI*180 <<std::endl;

	// std::vector<std::vector<float>> obstacle(100, std::vector<float>(2));
	// obstacle[0][0] = 1;
	// obstacle[0][1] = 2;
	// obstacle[1][0] = 3;
	// obstacle[1][1] = 4;
	// rotate(obstacle.begin(), obstacle.begin() + 1, obstacle.end());
	// std::cout<< obstacle[99][0] <<std::endl;
	
	// geometry_msgs::Vector3 pot;
	// pot.x = std::numeric_limits<double>::quiet_NaN();
	// pot.y = std::numeric_limits<double>::quiet_NaN();
	// std::cout<< pot.x <<std::endl;

	//-π~π
	// std::cout<< atan2(sqrt(3),1)/M_PI*180 <<std::endl;	//60deg　第1象限
	// std::cout<< atan2(sqrt(3),-1)/M_PI*180 <<std::endl;	//120deg　第2象限
	// std::cout<< atan2(-sqrt(3),-1)/M_PI*180 <<std::endl;	//-120deg　第3象限
	// std::cout<< atan2(-sqrt(3),1)/M_PI*180 <<std::endl;	//-60deg　第4象限

	// double roll,pitch,yaw;//[rad]
	// tf::Quaternion quaternion=tf::createQuaternionFromRPY(roll,pitch,yaw);
	// std::cout<< quaternion.x <<std::endl;

	// tf::Quaternion quat;//入力値
	// double r,p,y;//出力値
	//tf::Matrix3x3(quat).getRPY(r, p, y);//クォータニオン→オイラー角

	double roll  = 0*M_PI/180.0;
	double pitch = 0*M_PI/180.0;
	double yaw   = 370*M_PI/180.0;

	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
	
	//Eigen::Quaternionf quat(m);
	Eigen::Quaternionf quat(-0.6532814824,0.2705980501,-0.6532814824,0.2705980501);

	//std::cout << m << std::endl;

	std::cout << "x:"<<quat.x() << std::endl;
	std::cout << "y:"<<quat.y() << std::endl;
	std::cout << "z:"<<quat.z() << std::endl;
	std::cout << "w:"<<quat.w() << std::endl;

	Eigen::Matrix3f m2=quat.toRotationMatrix();
	Eigen::Vector3f ea = m2.eulerAngles(0, 1, 2); 
	std::cout<<ea(0)*180.0/M_PI<<",";//roll
	std::cout<<ea(1)*180.0/M_PI<<",";//pitch
	std::cout<<ea(2)*180.0/M_PI<<std::endl;//yaw

	// std::vector<double> path_x = {0,1,2,4,5};
	// std::vector<double> path_y = {0,2,1,4,1};
	// std::vector<geometry_msgs::Vector3> robot_path;

	// int size = path_x.size();
	// robot_path.resize(size);

	// for (int i = 0; i < size; i++)
	// {
	// 	robot_path[i].x = path_x[i];
	// 	robot_path[i].y = path_y[i];
	// }

	// spline(robot_path);

	// std::cout<< "xx = [";
	// for (int i = 0; i < robot_path.size(); i++)
	// {
	// 	std::cout<< robot_path[i].x <<" ";
	// }
	// std::cout<< "];" << std::endl;

	// std::cout<< "yy = [";
	// for (int i = 0; i < robot_path.size(); i++)
	// {
	// 	std::cout<< robot_path[i].y <<" ";
	// }
	// std::cout<< "];" << std::endl;

	// std::cout << "x=" << std::endl;
	// std::cout << x.transpose() << std::endl;

	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());

	// 0.0以上1.0未満の値を等確率で発生させる
	//std::uniform_real_distribution<> dist(0.0, 1.0);

	// 平均0.0、標準偏差1.0で分布させる
  	std::normal_distribution<> dist(0.0, 1.0);

	std::cout << "result = [";
	for (std::size_t n = 0; n < 1000; ++n) 
	{
		// 一様実数分布で乱数を生成する
		double result = dist(engine);

		std::cout << result << " ";
	}
	std::cout << "]" << std::endl;

    //ros::spin();

	return 0;
}

