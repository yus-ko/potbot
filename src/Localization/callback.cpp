#include<potbot/Localization.h>

void LocalizationClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    header_.frame_id = "/map";
    header_.stamp = ros::Time::now();
    encoder_value_ = msg;
    manage();
}

void LocalizationClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    //encoder_value.header = msg.header;
    odom_.pose = msg.pose;
    odom_.twist = msg.twist;
    encoder_value_ = msg.twist.twist;
    header_ = msg.header;
    header_.frame_id = "/map";
    //std::cout<<msg.header<<std::endl;
    manage();
}

void LocalizationClass::scan_callback(const sensor_msgs::LaserScan& msg)
{
    std::vector<std::vector<double>> color = {
                                                {1,0,0},
                                                {0,1,0},
                                                {0,0,1},
                                                {1,1,0},
                                                {1,0,1},
                                                {0,1,1},
                                                {0,0,0}
                                            };
    //ROS_INFO("scan callback");

    scan_ = msg;

    scan_.header.frame_id = "/lidar";
    pub_scan0_.publish(scan_);
    __MedianFilter(scan_);
    pub_scan1_.publish(scan_);
    std::vector<SEGMENT> segments;
    __Segmentation(scan_, segments);
    __SplitSegments(segments);
    

    visualization_msgs::MarkerArray seg;
    int id = 0;
    for (int i = 0; i < segments.size(); i++)
    {

        visualization_msgs::Marker segment;
        segment.header = scan_.header;
        segment.header.frame_id = "lidar";

        segment.ns = "segments_display";
        segment.id = i;
        segment.lifetime = ros::Duration(1);

        segment.type = segments[i].type;
        segment.action = visualization_msgs::Marker::MODIFY;

        
        segment.pose.position.x = segments[i].x;
        segment.pose.position.y = segments[i].y;
        segment.pose.position.z = 0;

        segment.pose.orientation.x = 0;
        segment.pose.orientation.y = 0;
        segment.pose.orientation.z = 0;
        segment.pose.orientation.w = 1;

        if (segment.type == visualization_msgs::Marker::SPHERE)
        {
            segment.scale.x = segments[i].radius*2;
            segment.scale.y = segments[i].radius*2;
        }
        else if (segment.type == visualization_msgs::Marker::CUBE)
        {
            segment.scale.x = segments[i].width;
            segment.scale.y = segments[i].height;
        }

        segment.scale.z = 0.001;

        segment.color.a = 0.3;

        segment.color.r = color[i%color.size()][0];
        segment.color.g = color[i%color.size()][1];
        segment.color.b = color[i%color.size()][2];
        
        seg.markers.push_back(segment);

        for (int j = 0; j < segments[i].points.size(); j++)
        {
            visualization_msgs::Marker point;
            point.header = segment.header;

            point.ns = "points_display";
            point.id = id++;
            point.lifetime = ros::Duration(1);

            point.type = segment.type;
            point.action = visualization_msgs::Marker::ADD;

            
            point.pose.position.x = segments[i].points[j].x;
            point.pose.position.y = segments[i].points[j].y;
            point.pose.position.z = 0;

            point.pose.orientation = segment.pose.orientation;

            point.scale.x = 0.02;
            point.scale.y = 0.02;
            point.scale.z = 0.001;

            point.color.a = 1;

            point.color.r = segment.color.r;
            point.color.g = segment.color.g;
            point.color.b = segment.color.b;
            
            seg.markers.push_back(point);
        }
        
    }
    pub_segment_.publish(seg);


    local_map_.header = header_;
    local_map_.header.frame_id = "/lidar";
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

    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::convert(odom_.pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    int size = scan_.ranges.size();
    for (int i = 0; i < size; i++)
    {
        if (!isinf(scan_.ranges[i]) && !isnan(scan_.ranges[i]))
        {
            // double angle = i * scan_.angle_increment + scan_.angle_min + yaw;
            double angle = i * scan_.angle_increment + scan_.angle_min;
            double distance = scan_.ranges[i] + scan_.range_min;
            // double x = distance * cos(angle) + odom_.pose.pose.position.x;
            // double y = distance * sin(angle) + odom_.pose.pose.position.y;
            double x = distance * cos(angle);
            double y = distance * sin(angle);
            //ROS_INFO("%f, %f, %f, %f, %d",x,y,local_map_.info.origin.position.x,local_map_.info.origin.position.y, get_index(x,y,local_map_.info));
            local_map_.data[get_index(x,y,local_map_.info)] = 100;
        }
    }

    pub_localmap_.publish(local_map_);

}

void LocalizationClass::inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    initial_pose_ = msg;
    ROS_INFO("subscribe initial pose");
    //std::cout<< initial_pose_ <<std::endl;

    odom_.header = initial_pose_.header;
    odom_.pose = initial_pose_.pose;

    if (localization_method_id_ == PARTICLE_FILTER) set_pose(initial_pose_);

    puclish_odom();
}

void LocalizationClass::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal");
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

void LocalizationClass::cluster_callback(const potbot::ClassificationVelocityData& msg)
{
    pcl_cluster_ = msg;
    ROS_INFO("cluster");
}

void LocalizationClass::__param_callback(const potbot::LocalizationConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    Tn_ = param.threshold_point_num;
    square_width_ = param.squre_width;
}