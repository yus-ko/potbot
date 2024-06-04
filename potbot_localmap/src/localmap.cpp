#include<potbot_localmap/Localmap.h>

LocalmapClass::LocalmapClass(tf2_ros::Buffer& tf, const std::string& name) : tf_buffer_(tf)
{

    ros::NodeHandle n("~");
    n.getParam("frame_id_robot_base",           frame_id_robot_base_);
    n.getParam("apply_cluster_to_localmap",     apply_cluster_to_localmap_);
    n.getParam("prediction_time",               prediction_time_);
    n.getParam("max_estimated_linear_velocity", max_estimated_linear_velocity_);
    n.getParam("max_estimated_angular_velocity",max_estimated_angular_velocity_);

	sub_obstacles_scan_	= nhSub_.subscribe("obstacle/scan/estimate",1,&LocalmapClass::__obstacles_scan_callback,this);
	sub_obstacles_pcl_	= nhSub_.subscribe("obstacle/pcl",1,&LocalmapClass::__obstacles_pcl_callback,this);

	pub_localmap_		= nhPub_.advertise<nav_msgs::OccupancyGrid>("Localmap", 1);

    dsrv_ = new dynamic_reconfigure::Server<potbot_localmap::LocalmapConfig>(ros::NodeHandle("~/" + name));
    dynamic_reconfigure::Server<potbot_localmap::LocalmapConfig>::CallbackType cb = boost::bind(&LocalmapClass::__param_callback, this, _1, _2);
    dsrv_->setCallback(cb);
	
}

void LocalmapClass::__obstacles_scan_callback(const potbot_msgs::ObstacleArray& msg)
{
    obstacles_scan_                 = msg;

    nav_msgs::OccupancyGrid local_map;

    local_map.header                = obstacles_scan_.header;
    local_map.info.map_load_time    = local_map.header.stamp;
    local_map.info.width            = 240;
    local_map.info.height           = 240;
    local_map.info.resolution       = 0.05;

    geometry_msgs::PoseStamped robot_pose = potbot_lib::utility::get_Pose_from_tf(tf_buffer_, local_map.header.frame_id, frame_id_robot_base_);

    geometry_msgs::Pose origin;
    origin.position.x               = robot_pose.pose.position.x - (local_map.info.width*local_map.info.resolution/2.0);
    origin.position.y               = robot_pose.pose.position.y - (local_map.info.height*local_map.info.resolution/2.0);
    local_map.info.origin           = origin;
    
    int mapsize                     = local_map.info.width*local_map.info.height;

    local_map.data.resize(mapsize);

    static ros::Time local_map_stamp_pre = local_map.header.stamp - ros::Duration(100);
    double_t dt = local_map.header.stamp.toSec() - local_map_stamp_pre.toSec();
    // dt = 0.2;

    for (const auto& obstacle : obstacles_scan_.data)
    {
        double v                    = obstacle.twist.linear.x;  //障害物の並進速度
        double omega                = fmod(obstacle.twist.angular.z, 2*M_PI); //障害物の回転角速度
        double yaw                  = potbot_lib::utility::get_Yaw(obstacle.pose.orientation);  //障害物の姿勢
        double width                = obstacle.scale.y; //障害物の幅
        double depth                = obstacle.scale.x; //障害物の奥行き
        double size                 = width + depth;

        // v=0.2;omega=0;yaw=1.57;

        if (size < apply_cluster_to_localmap_)
        {
            ROS_INFO("est vel %d: %f, %f, %f, %f, %f", obstacle.id, obstacle.pose.position.x, obstacle.pose.position.y, yaw, v, omega);
            if (abs(v) < max_estimated_linear_velocity_ && abs(omega) < max_estimated_angular_velocity_)
            {
                //並進速度と角速度を一定として1秒後までの位置x,yを算出
                for (double t = 0; t < prediction_time_; t += dt)
                {
                    double distance = v*t;
                    double angle = omega*t + yaw;
                    for (const auto& p : obstacle.points)
                    {
                        double x            = distance*cos(angle) + p.x;
                        double y            = distance*sin(angle) + p.y;
                        local_map.data[potbot_lib::utility::get_MapIndex(x, y, local_map.info)] = 100;
                    }
                    
                }
            }
            else
            {
                local_map.data[potbot_lib::utility::get_MapIndex(obstacle.pose.position.x, obstacle.pose.position.y, local_map.info)] = 100;
            }
        }

        for (const auto& point : obstacle.points)
        {
            local_map.data[potbot_lib::utility::get_MapIndex(point.x, point.y, local_map.info)] = 100;
        }
        
    }
    local_map_stamp_pre = local_map.header.stamp;

    pub_localmap_.publish(local_map);
}

void LocalmapClass::__obstacles_pcl_callback(const potbot_msgs::ObstacleArray& msg)
{
    
}

void LocalmapClass::__param_callback(const potbot_localmap::LocalmapConfig& param, uint32_t level)
{
    apply_cluster_to_localmap_  = param.apply_localmap_threshold_2d_size;
    prediction_time_            = param.prediction_time;
    max_estimated_linear_velocity_ = param.max_estimated_linear_velocity;
    max_estimated_angular_velocity_ = param.max_estimated_angular_velocity;
}