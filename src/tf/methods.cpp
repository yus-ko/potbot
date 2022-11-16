#include<autonomous_mobile_robot_2022/tf.h>


void TFClass::pwcs_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    pwcs_msg = msg;
}


void TFClass::scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan_msg = msg;
}

void TFClass::get_topic()
{
    //時間
    manage_time_pre = manage_time;
    manage_time = ros::Time::now();

	geometry_msgs::TransformStamped state;

    //ロボット位置
	state.header.stamp = manage_time;

	state.header.frame_id = "world";
	state.child_frame_id  = "robot";

    state.transform.translation.x = pwcs_msg.pose.pose.position.x;
    state.transform.translation.y = pwcs_msg.pose.pose.position.y;
    state.transform.translation.z = pwcs_msg.pose.pose.position.z;
    state.transform.rotation = pwcs_msg.pose.pose.orientation;

	robotState_broadcaster.sendTransform(state);
    
    //レーザーレンジファインダー位置
    state.header.frame_id = "robot";
	state.child_frame_id  = "LRF";

	state.transform.translation.x = 0.0;
	state.transform.translation.y = 0.0;
	state.transform.translation.z = 0.0;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0);
	state.transform.rotation = quat;

    LRF_broadcaster.sendTransform(state);

    //ステレオカメラ位置
    state.header.frame_id = "robot";
	state.child_frame_id  = "StereoCamera";

	state.transform.translation.x = 0.0;
	state.transform.translation.y = 0.0;
	state.transform.translation.z = 0.0;

    quat = tf::createQuaternionMsgFromYaw(0);
	state.transform.rotation = quat;

    StereoCamera_broadcaster.sendTransform(state);

}

void TFClass::manage()
{
    //std::cout<< "----------------------------------------" <<std::endl;
    get_topic();
    
}
