#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
//#include <beego_control/beego_encoder.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/Geometry> //EigenのGeometry関連の関数を使う場合，これが必要

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

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_te");

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

    //ros::spin();

	return 0;
}

