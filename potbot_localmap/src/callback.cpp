#include<potbot_localmap/Localmap.h>

void LocalmapClass::__obstacles_scan_callback(const potbot_msgs::ObstacleArray& msg)
{
    obstacles_scan_                 = msg;

    nav_msgs::OccupancyGrid local_map;

    local_map.header                = obstacles_scan_.header;
    local_map.info.map_load_time    = local_map.header.stamp;
    local_map.info.width            = 240;
    local_map.info.height           = 240;
    local_map.info.resolution       = 0.05;

    geometry_msgs::Pose origin;
    origin.position.x               = -(local_map.info.width*local_map.info.resolution/2.0);
    origin.position.y               = -(local_map.info.height*local_map.info.resolution/2.0);
    local_map.info.origin           = origin;
    
    int mapsize                     = local_map.info.width*local_map.info.height;

    local_map.data.resize(mapsize);

    for (const auto& obstacle : obstacles_scan_.data)
    {
        double v                    = obstacle.twist.linear.x;  //障害物の並進速度
        double omega                = obstacle.twist.angular.z; //障害物の回転角速度
        double yaw                  = potbot_lib::utility::get_Yaw(obstacle.pose.orientation);  //障害物の姿勢
        double width                = obstacle.scale.y; //障害物の幅
        double depth                = obstacle.scale.x; //障害物の奥行き
        double size                 = width + depth;

        // ROS_INFO("size:%f",size);

        if (size > apply_cluster_to_localmap_)
        {
            if (abs(v) > 0.1 && abs(v) < 2.0 && abs(omega) < 1)
            {
                //並進速度と角速度を一定として1秒後までの位置x,yを算出
                double dt = 0.1;
                for (double t = 0; t <= 1; t += dt)
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

    pub_localmap_.publish(local_map);
}

void LocalmapClass::__obstacles_pcl_callback(const potbot_msgs::ObstacleArray& msg)
{
    
}

void LocalmapClass::__param_callback(const potbot_localmap::LocalmapConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    apply_cluster_to_localmap_ = param.apply_localmap_threshold_2d_size;
}