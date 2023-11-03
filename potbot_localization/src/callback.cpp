#include<potbot_localization/Localization.h>

// void LocalizationClass::encoder_callback(const geometry_msgs::Twist& msg)
// {
//     header_.frame_id = "/" + FRAME_ID_GLOBAL;
//     header_.stamp = ros::Time::now();
//     encoder_value_ = msg;
//     manage();
// }

void LocalizationClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    //encoder_value.header = msg.header;
    odom_.pose = msg.pose;
    odom_.twist = msg.twist;
    encoder_value_ = msg.twist.twist;
    header_ = msg.header;
    //std::cout<<msg.header<<std::endl;
    manage();
}

// void LocalizationClass::beego_encoder_callback(const potbot_msgs::beego_encoder& msg)
// {
//     header_ = msg.header;
//     // odom_.pose.pose.position.x = 0; //add
//     // odom_.pose.pose.position.y = 0; //add

//     // double yaw = 0; //add
//     // geometry_msgs::Quaternion quat;
//     // getQuat(0, 0, yaw, quat);
//     // odom_.pose.pose.orientation = quat;

//     // odom_.twist.twist.linear.x = 0;     //add
//     // odom_.twist.twist.angular.z = 0;    //add

//     double wheel_d = 0.275;
//     double v = (-msg.vel.r+msg.vel.l)/2.0;
// 	double omega = (-msg.vel.r-msg.vel.l)/(wheel_d);

//     encoder_value_.linear.x = v;        //add
//     encoder_value_.angular.z = omega;   //add

//     manage();
// }

void LocalizationClass::inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    initial_pose_ = msg;
    ROS_INFO("subscribe initial pose");
    //std::cout<< initial_pose_ <<std::endl;

    odom_.header = initial_pose_.header;
    odom_.pose = initial_pose_.pose;

    if (localization_method_id_ == potbot_lib::PARTICLE_FILTER) set_pose(initial_pose_);

    puclish_odom();
}

void LocalizationClass::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal: localization");
    //std::cout<< goal_ <<std::endl;
}

void LocalizationClass::point_callback(const geometry_msgs::PointStamped& msg)
{
    point_ = msg;
    ROS_INFO("subscribe point");
    //std::cout<< point_ <<std::endl;
}

void LocalizationClass::map_callback(const nav_msgs::OccupancyGrid& msg)
{
    world_map_ = msg;
    //std::cout<< world_map_.info <<std::endl;
    //std::cout<< world_map_.data.size() <<std::endl;
    //ROS_INFO("%d",world_map_.data[10000]);

    // bool print = false;
    // for (int i = 0;i < world_map_.data.size(); i++)
    // {
    //     if(!print && world_map_.data[i] == 100)
    //     {
    //         //std::cout<< i << ",";
    //         //printf("(%f, %f)",(i%608)*0.05 + 5.2,(i/608)*0.05 - 3);
    //         geometry_msgs::Point p = get_coordinate(i);
    //         printf("(%f, %f)",p.x,p.y);
    //         printf("(%d,%d)",i,get_index(p.x,p.y));
    //         print = true;
    //     }
    //     //if(i%608 == 0) std::cout<<std::endl;
    // }

    // for (double i = -2; i <= 2; i+=0.05)
    // {
    //     for (double j = -2; j <= 2; j+=0.05) 
    //     {
    //         double x = -9.5 + j;
    //         double y = -0.5 + i ;
    //         int data = world_map_.data[get_index(x,y)];
    //         if (data == 100) ROS_INFO("(%f, %f): %d",x,y,data);
    //     }
    // }

}

void LocalizationClass::__scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;

    local_map_.header = header_;
    local_map_.info = world_map_.info;
    local_map_.info.map_load_time = header_.stamp;
    local_map_.info.width = 240;
    local_map_.info.height = 240;
    local_map_.info.resolution = 0.05;

    //ROS_INFO("maximum likefood particle:%d",maximum_likefood_particle_id_);
    geometry_msgs::Pose origin;
    // origin.position.x = -(local_map_.info.width*local_map_.info.resolution/2) + odom_.pose.pose.position.x;
    // origin.position.y = -(local_map_.info.height*local_map_.info.resolution/2) + odom_.pose.pose.position.y;
    origin.position.x = -(local_map_.info.width*local_map_.info.resolution/2);
    origin.position.y = -(local_map_.info.height*local_map_.info.resolution/2);
    local_map_.info.origin = origin;
    
    int mapsize = local_map_.info.width*local_map_.info.height;

    local_map_.data.resize(0);
    local_map_.data.resize(mapsize);

    // double roll, pitch, yaw;
    // tf2::Quaternion quat;
    // tf2::convert(odom_.pose.pose.orientation, quat);
    // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    int size = scan_.ranges.size();
    //ROS_INFO("range raw: %f, min: %f",scan_.ranges[540], scan_.range_min);
    for (int i = 0; i < size; i++)
    {
        //if (!std::isinf(scan_.ranges[i]) && !std::isnan(scan_.ranges[i]))
        if (scan_.range_min <= scan_.ranges[i] && scan_.ranges[i] <= scan_.range_max)
        {
            // double angle = i * scan_.angle_increment + scan_.angle_min + yaw;
            double angle = i * scan_.angle_increment + scan_.angle_min;
            double distance = scan_.ranges[i] + scan_.range_min;
            // double x = distance * cos(angle) + odom_.pose.pose.position.x;
            // double y = distance * sin(angle) + odom_.pose.pose.position.y;
            double x = distance * cos(angle);
            double y = distance * sin(angle);
            // ROS_INFO("%f, %f, %f, %f, %d",x,y,local_map_.info.origin.position.x,local_map_.info.origin.position.y, potbot_lib::utility::get_index(x,y,local_map_.info));
            local_map_.data[potbot_lib::utility::get_MapIndex(x,y,local_map_.info)] = 100;
        }
    }

    pub_localmap_.publish(local_map_);
}

void LocalizationClass::__param_callback(const potbot_localization::LocalizationConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    Tn_ = param.threshold_point_num;
    square_width_ = param.squre_width;
}