#include<potbot/PathPlanning.h>

void PathPlanningClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    encoder_value = msg;
    encoder_first = true;
}

void PathPlanningClass::pwcs_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    pwcs_msg = msg;
    odom.pose.pose.position = msg.pose.pose.position;

    Eigen::Quaternionf quat(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);

    Eigen::Matrix3f m2=quat.toRotationMatrix();
	Eigen::Vector3f ea = m2.eulerAngles(1, 2, 0); 
	// std::cout<<ea(0)*180.0/M_PI<<",";//roll
	// std::cout<<ea(1)*180.0/M_PI<<",";//pitch
	// std::cout<<ea(2)*180.0/M_PI<<std::endl;//yaw

    odom.pose.pose.orientation.z = ea(2);

    // std::cout<< "pwcs_callback = "<<std::endl;
    // std::cout<< odom.pose.pose <<std::endl;
    encoder_first = true;
}

void PathPlanningClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    return;
    header_ = msg.header;
    odom_msg = msg;
    odom = msg;
    //odom_ = msg;
    //ROS_INFO("%s",odom_.header.frame_id.c_str());
    encoder_value = msg.twist.twist;
    encoder_first = true;
}

void PathPlanningClass::scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan = msg;
    scan_first = true;
}

void PathPlanningClass::coefficient_callback(const std_msgs::Float32& msg)
{
    coe_0 = msg.data;
}

void PathPlanningClass::cluster_callback(const potbot::ClassificationVelocityData& msg)
{
    pcl_cluster = msg;
}

void PathPlanningClass::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal");
    //std::cout<< goal_ <<std::endl;
    pub_goal_.publish(goal_);
}

void PathPlanningClass::local_map_callback(const nav_msgs::OccupancyGrid& msg)
{
    local_map_ = msg;
    // ROS_INFO("subscribe local map");
    static int cnt = -1;
    static ros::Time transform_time;
    if (cnt == -1)
    {
        transform_time = local_map_.header.stamp;
    }
    if (cnt++ > 10)
    {
        cnt = 0;
        tf2_ros::TransformListener tf_listener(tf_buffer_);

        std::string target_frame = "lidar";
        std::string source_frame = "robot";

        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tf_buffer_.lookupTransform(target_frame, source_frame, transform_time);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("TF2 exception: " << ex.what());
            return;
        }

        // 変換する座標
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header.frame_id = source_frame;
        pose_in.header.stamp = transform_time;
        local_map_.info.origin;
        pose_in.pose = local_map_.info.origin;

        tf2::doTransform(pose_in, pose_out, transformStamped);

        // 変換後の座標を表示
        // ROS_INFO("Transformed point: (%f, %f, %f) in frame %s at time %f", 
        //         pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z,
        //         pose_out.header.frame_id.c_str(), pose_out.header.stamp.toSec());
    }
}

 void PathPlanningClass::__param_callback(const potbot::PathPlanningConfig& param, uint32_t level)
 {
    // ROS_INFO("%d",level);
    rho_zero_ = param.rho_zero;
    eta_ = param.eta;
    kp_ = param.kp;
 }