#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class test{

    private:
		ros::NodeHandle nhSub;
		ros::Subscriber sub_encoder;

		ros::NodeHandle nhPub;
        ros::Publisher pub_cmd, pub_odom;

        ros::Time encoder_time_pre;
        nav_msgs::Odometry odom;
        double bottom_v,bottom_omega;

        bool encoder_first = true;

        ros::WallTime start_time;
        geometry_msgs::Twist cmd;

    public:
        test();
        ~test();
	    void encoder_callback(const nav_msgs::Odometry& msg);
};
