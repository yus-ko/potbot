#include<potbot_rviz/rviz.h>

void rvizClass::odom_callback(const nav_msgs::Odometry& msg)
{
    odom = msg;
    CreateTraj();
    publishTraj();
}

void rvizClass::manage()
{
    
}

bool is_equal(geometry_msgs::Vector3 A,geometry_msgs::Vector3 B)
{
    if (A.x == B.x && A.y == B.y && A.z == B.z)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void rvizClass::CreateTraj()
{
    ros::Time now = ros::Time::now();
    robot_traj_.header = odom.header;
    robot_traj_.header.frame_id = "/map";

    int size = 0;
    if (now.toSec() < CreatePath_time_pre.toSec() + 1) size = robot_traj_.poses.size(); 
    robot_traj_.poses.resize(size+1);
    robot_traj_.poses[size].header = odom.header;
    robot_traj_.poses[size].pose = odom.pose.pose;

    CreatePath_time_pre = now;
}

void rvizClass::publishTraj()
{
    pub_path.publish(robot_traj_);
}