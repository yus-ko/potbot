#include<potbot_controller/Controller.h>

ControllerClass::ControllerClass()
{
	ros::NodeHandle n("~");
    n.getParam("publish_control_command",   publish_command_);
    n.getParam("topic_odom",                topic_odom_);
    n.getParam("topic_cmd",                 topic_cmd_);
    n.getParam("topic_goal",                topic_goal_);

	sub_odom_			= nhSub_.subscribe(topic_odom_,						1,&ControllerClass::__odom_callback,this);
	sub_path_			= nhSub_.subscribe("Path",							1,&ControllerClass::__path_callback,this);
	sub_goal_			= nhSub_.subscribe(topic_goal_,						1,&ControllerClass::__goal_callback, this);

	pub_path_request_	= nhPub_.advertise<std_msgs::Empty>(				"create_path", 1);
	pub_cmd_			= nhPub_.advertise<geometry_msgs::Twist>(			topic_cmd_, 1);
	pub_look_ahead_		= nhPub_.advertise<visualization_msgs::Marker>(		"LookAhead", 1);

	f_ = boost::bind(&ControllerClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

	static tf2_ros::TransformListener tfListener(tf_buffer_);

}
ControllerClass::~ControllerClass(){
}

void ControllerClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
    robot_controller_.set_msg(odom_);
    manage();
}

void ControllerClass::__path_callback(const nav_msgs::Path& msg)
{
    ROS_INFO("subscribe path: controller");
    if (msg.header.frame_id != odom_.header.frame_id)
    {
        nav_msgs::Path init;
        robot_path_ = init;
        potbot_lib::utility::get_tf(tf_buffer_, msg, odom_.header.frame_id, robot_path_);
    }
    else
    {
        robot_path_ = msg;
    }
    robot_controller_.set_target_path(robot_path_);
}

void ControllerClass::__goal_callback(const geometry_msgs::PoseStamped& msg)
{
    ROS_INFO("subscribe goal: controller");
	if (msg.header.frame_id != odom_.header.frame_id)
    {
        goal_ = potbot_lib::utility::get_tf(tf_buffer_, msg, odom_.header.frame_id);
    }
    else
    {
        goal_ = msg;
    }
	robot_controller_.set_target(goal_.pose);
    // __publish_path_request();
}

void ControllerClass::__param_callback(const potbot_msgs::ControllerConfig& param, uint32_t level)
{
    publish_command_             		= param.publish_control_command;
	stop_margin_angle_					= param.stop_margin_angle;
	stop_margin_distance_				= param.stop_margin_distance;
	distance_to_lookahead_point_		= param.distance_to_lookahead_point;
	distance_change_to_pose_alignment_	= param.distance_change_to_pose_alignment; 

    robot_controller_.set_gain(	    param.gain_p,
						            param.gain_i,
						            param.gain_d);

	robot_controller_.set_margin(	stop_margin_angle_,
						            stop_margin_distance_);

	robot_controller_.set_limit(	param.max_linear_velocity,
						            param.max_angular_velocity);
	
	robot_controller_.set_distance_to_lookahead_point(distance_to_lookahead_point_);

    robot_controller_.set_initialize_pose(param.initialize_pose);
}

void ControllerClass::manage()
{
    controller();
    if (publish_command_) __publishcmd();
}

void ControllerClass::controller()
{

    double distance = potbot_lib::utility::get_Distance(odom_.pose.pose.position, goal_.pose.position);
    double angle = potbot_lib::utility::get_Yaw(goal_.pose.orientation) - potbot_lib::utility::get_Yaw(odom_.pose.pose.orientation);
    // ROS_INFO("%f / %f, %f / %f", distance, stop_margin_distance_, angle, stop_margin_angle_);
    if (distance < stop_margin_distance_ && abs(angle) < stop_margin_angle_)
    {
        ROS_INFO("reach the goal");
        geometry_msgs::Twist cmd;
        cmd_=cmd;
		set_goal_ = false;
    }
    else if (distance < distance_change_to_pose_alignment_)
    {
		if (!set_goal_)
		{
			set_goal_ = true;
			robot_controller_.set_target(goal_.pose);
		}
        __PoseAlignment();
    }
    else
    {   
        if (!robot_path_.poses.empty()) __LineFollowing();
    }
}

void ControllerClass::__LineFollowing()
{
    ROS_INFO("controller: line following");
    robot_controller_.pure_pursuit();

	nav_msgs::Odometry robot_pose;
	robot_controller_.to_msg(robot_pose);
	
	visualization_msgs::Marker lookahead_msg;
	robot_controller_.get_lookahead(lookahead_msg);
	lookahead_msg.header = odom_.header;

	cmd_ = robot_pose.twist.twist;
	pub_look_ahead_.publish(lookahead_msg);

	if (potbot_lib::utility::get_Distance(odom_.pose.pose.position, robot_path_.poses.back().pose.position) <= distance_to_lookahead_point_-0.01 ||
		potbot_lib::utility::get_Distance(odom_.pose.pose.position, lookahead_msg.pose.position)			>= distance_to_lookahead_point_+1 ) __publish_path_request();
}

void ControllerClass::__PoseAlignment()
{
    ROS_INFO("controller: pose alignment");
	robot_controller_.pid_control();
	nav_msgs::Odometry robot_pose;
	robot_controller_.to_msg(robot_pose);
	cmd_ = robot_pose.twist.twist;
}

void ControllerClass::__publishcmd()
{
    pub_cmd_.publish(cmd_);
}

void ControllerClass::__publish_path_request()
{
    geometry_msgs::PoseStamped goal_init;
    if(goal_init != goal_)
    {
        std_msgs::Empty empty;
        pub_path_request_.publish(empty);
    }
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_co");

    ControllerClass cc;
	ros::spin();

	return 0;
}