#include<potbot/Localization.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

double LocalizationClass::__Median(std::vector<double> v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    if (v.size() % 2 == 1) 
    {
        return v[n];
    } else 
    {
        std::nth_element(v.begin(), v.begin() + n - 1, v.end());
        return (v[n - 1] + v[n]) / 2.0;
    }
}

void LocalizationClass::__MedianFilter(sensor_msgs::LaserScan &scan)
{
    int window_num = 3;
    scans_.push_back(scan);
    int scans_size = scans_.size();
    if (scans_size >= window_num)
    {
        int scan_size = scan.ranges.size();
        for (int i = 1; i < scan_size-1; i++)
        {
            std::vector<double> scan_data;
            for (int t = scans_size - window_num; t < scans_size; t++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    double data = scans_[t].ranges[i+j];
                    if (isinf(data) || isnan(data))
                    {
                        data = std::numeric_limits<double>::infinity();
                    }
                    scan_data.push_back(data);
                }
            }
            scan.ranges[i] = __Median(scan_data);
        }
        scans_.erase(scans_.begin());
    }
}

void LocalizationClass::__Segmentation(sensor_msgs::LaserScan &scan, std::vector<SEGMENT> &segments)
{
    int size = scan.ranges.size();
    bool start = false;
    SEGMENT seg;
    seg.type = visualization_msgs::Marker::SPHERE;
    for (int i = 0; i < size; i++)
    {
        
        double data = scan.ranges[i];
        if (!isinf(data) && !isnan(data))
        {
            if(!start)
            {
                start = true;
                seg.points.resize(0);
            }
            POINT p;
            p.index = i;
            p.theta = p.index * scan_.angle_increment + scan_.angle_min;
            p.r = scan_.ranges[i] + scan_.range_min;
            p.x = p.r * cos(p.theta);
            p.y = p.r * sin(p.theta);

            if (seg.points.size() > 0)
            {    
                POINT &p_pre = seg.points.back();

                double distance = sqrt(pow(p.x - p_pre.x,2) + pow(p.y - p_pre.y,2));
                if (distance <= 0.3)
                {
                    seg.points.push_back(p);
                }
                else
                {
                    start = false;
                    segments.push_back(seg);
                    i--;
                    continue;
                }
            }
            else
            {
                seg.points.push_back(p);
            }

            if (i == size - 1 && start)
            {
                segments.push_back(seg);
            }
        }
        else if(start)
        {
            start = false;
            segments.push_back(seg);
        }
    }
}

double LocalizationClass::__distanceToLineSegment(POINT o, POINT p, POINT q)
{
    // double ABx = q.x - p.x;
    // double ABy = q.y - p.y;
    // double ABlength = sqrt(pow(ABx, 2) + pow(ABy, 2));
    // double ABx_norm = ABx / ABlength;
    // double ABy_norm = ABy / ABlength;
    // double APx = o.x - p.x;
    // double APy = o.y - p.y;
    // double APdistance = sqrt(pow(APx, 2) + pow(APy, 2));
    // double dotProduct = APx * ABx_norm + APy * ABy_norm;
    // double xd = p.x + dotProduct * ABx_norm;
    // double yd = p.y + dotProduct * ABy_norm;
    // double distance = sqrt(pow(o.x - xd, 2) + pow(o.y - yd, 2));

    // double a = (q.y - p.y)/(q.x - p.x);
    // double b = 1;
    // double c = -p.y;
    // double distance = abs(o.x*a + b*o.y + c) / sqrt(a*a+b*b);

    double theta = atan2(o.y-p.y, o.x-p.x) - atan2(q.y-p.y, q.x-p.x);
    // double theta = acos((q.x*p.x + q.y*p.y) / (sqrt(q.x*q.x + q.y*q.y) * sqrt(p.x*p.x + p.y*p.y)));
    double l = sqrt(pow(o.x - p.x, 2) + pow(o.y - p.y, 2));
    double distance = l*sin(theta);

    return distance;
}

void LocalizationClass::__SplitSegments(std::vector<SEGMENT> &segments)
{
    std::vector<SEGMENT> segments_original = segments;   //Vc
    segments.resize(0); //Vresult

    
    double square_width = square_width_;

    while(segments_original.size() != 0)
    {
        int Nc0 = segments_original[0].points.size();
        if (Nc0 > 2)
        {
            POINT p = segments_original[0].points.front();
            POINT q = segments_original[0].points.back();
            std::vector<double> distance;
            for (int i = 1; i < Nc0-1; i++)
            {
                double d = __distanceToLineSegment(segments_original[0].points[i], p, q);
                distance.push_back(d);
            }
            std::vector<double>::iterator max_itr = std::max_element(distance.begin(), distance.end());
            double Dm = *max_itr;
            double S = sqrt(pow(q.x - p.x,2) + pow(q.y - p.y,2));

            segments_original[0].x = (p.x + q.x)/2;
            segments_original[0].y = (p.y + q.y)/2;

            if (Dm > square_width*S)
            {
                segments_original[0].type = visualization_msgs::Marker::SPHERE;
                segments_original[0].radius = S/2;
                segments.push_back(segments_original[0]);
                segments_original.erase(segments_original.begin());
            }
            else
            {
                segments_original[0].type = visualization_msgs::Marker::CUBE;
                segments_original[0].width = abs(q.x - p.x);
                segments_original[0].height = abs(q.y - p.y);
                segments.push_back(segments_original[0]);
                segments_original.erase(segments_original.begin());
            }
        }
        else
        {
            segments_original.erase(segments_original.begin());
        }


    }

}

// void LocalizationClass::__SplitSegments(std::vector<SEGMENT> &segments)
// {
//     std::vector<SEGMENT> segments_original = segments;   //Vc
//     segments.resize(0); //Vresult

    
//     double square_width = square_width_;

//     while(segments_original.size() != 0)
//     {
//         int Nc0 = segments_original[0].points.size();
//         int Tn = Tn_/segments_original[0].points[0].r;
//         if (Nc0 > Tn)
//         {
//             //Calculate Dm of Vc[0] nad get pk that corresponds to Dm
//             POINT p = *segments_original[0].points.begin();
//             int last_index = Nc0-1;
//             POINT q = segments_original[0].points[last_index];
//             std::vector<double> distance;
//             for (int i = 1; i < Nc0-1; i++)
//             {
//                 double d = __distanceToLineSegment(segments_original[0].points[i], p, q);
//                 distance.push_back(d);
//             }
//             std::vector<double>::iterator max_itr = std::max_element(distance.begin(), distance.end());
//             double Dm = *max_itr;
//             int k = std::distance(distance.begin(), max_itr) + 1;
//             int n = distance.size() + 2;
//             double S = sqrt(pow(q.x - p.x,2) + pow(q.y - p.y,2));

//             if (Dm > square_width*S)
//             {
//                 SEGMENT B1,B2;
//                 std::vector<POINT> b1(segments_original[0].points.begin(), segments_original[0].points.begin() + k);
//                 std::vector<POINT> b2(segments_original[0].points.begin() + k, segments_original[0].points.end());
//                 B1.points = b1;
//                 B2.points = b2;

//                 if (k > Tn)
//                 {
//                     B1.type = visualization_msgs::Marker::CUBE;
//                     segments_original.push_back(B1);
//                 }
//                 else
//                 {
//                     B1.type = visualization_msgs::Marker::SPHERE;
//                     segments.push_back(B1);
//                 }

//                 if (n - k > Tn)
//                 {
//                     B2.type = visualization_msgs::Marker::CUBE;
//                     segments_original.push_back(B2);
//                 }
//                 else
//                 {
//                     B2.type = visualization_msgs::Marker::SPHERE;
//                     segments.push_back(B2);
//                 }
//                 segments_original.erase(segments_original.begin());
//             }
//             else
//             {
//                 segments_original[0].type = visualization_msgs::Marker::CUBE;
//                 segments.push_back(segments_original[0]);
//                 segments_original.erase(segments_original.begin());
//             }
//         }
//         else
//         {
//             segments.push_back(segments_original[0]);
//             segments_original.erase(segments_original.begin());
//         }


//     }

//     for (int i = 0; i < segments.size(); i++)
//     {
//         if (segments[i].points.size() <= 3)
//         {
//             segments.erase(segments.begin() + i--);
//             continue;
//         }

//         POINT p = *segments[i].points.begin();
//         int last_index = segments[i].points.size()-1;
//         POINT q = segments[i].points[last_index];
//         segments[i].x = (p.x + q.x)/2;
//         segments[i].y = (p.y + q.y)/2;

//         if (segments[i].type == visualization_msgs::Marker::SPHERE)
//         {
//             segments[i].radius = sqrt(pow(q.x - p.x,2) + pow(q.y - p.y,2))/2;
//         }
//         else if (segments[i].type == visualization_msgs::Marker::CUBE)
//         {
//             segments[i].width = abs(q.x - p.x);
//             segments[i].height = abs(q.y - p.y);
//         }
//     }

// }

void LocalizationClass::set_pose(geometry_msgs::PoseWithCovarianceStamped pose)
{
    maximum_likefood_particle_id_ = 0;
    for (int i = 0; i < particle_num_; i++)
    {
        particles_.markers[i].header = header_;
        particles_.markers[i].ns = "Particle_display";
        particles_.markers[i].id = i;
        particles_.markers[i].lifetime = ros::Duration();

        particles_.markers[i].type = visualization_msgs::Marker::ARROW;
        particles_.markers[i].action = visualization_msgs::Marker::ADD;

        particles_.markers[i].pose.position = pose.pose.pose.position;
        particles_.markers[i].pose.orientation = pose.pose.pose.orientation;

        particles_.markers[i].scale.x = 0.5;
        particles_.markers[i].scale.y = 0.05;
        particles_.markers[i].scale.z = 0.05;

        particles_.markers[i].color.a = 1;

        particles_.markers[i].color.r = 1;
        particles_.markers[i].color.g = 1;
        particles_.markers[i].color.b = 0;
    }
}

double LocalizationClass::match_rate(nav_msgs::OccupancyGrid local,nav_msgs::OccupancyGrid world)
{
    //ROS_INFO("match rate");

    int local_map_size = local.info.width*local.info.height;
    int max = 0;
    int sum = 0;
    for(int local_map_index = 0; local_map_index < local_map_size; local_map_index++)
    {
        if (local.data[local_map_index] == 100)
        {
            max++;
            geometry_msgs::Point point = get_coordinate(local_map_index, local.info);
            int world_map_index = get_index(point.x, point.y, world.info);
            if (world.data[world_map_index] == 100) sum++;
        }
    }

    return double(sum) / double(max);
}

void LocalizationClass::manage()
{
    if (header_pre_.stamp.toSec() != 0.0)
    {
        odom_.header = header_;

        if (localization_method_id_ == PARTICLE_FILTER)
        {
            //パーティクル生成
            create_particle();

            //2秒毎にリサンプリング
            if (header_.stamp.toSec() > resampling_time_.stamp.toSec() + 2)
            {
                resampling_time_ = header_;
                resampling();
            }

            //尤度値の最も高いパーティクルを自己位置とする
            odom_.pose.pose = particles_.markers[maximum_likefood_particle_id_].pose;
            odom_.twist.twist = encoder_value_;
            
        }
        else if(localization_method_id_ == DEAD_RECKONING)
        {
            if (robot_id_ == MEGAROVER && !IS_SIMULATOR) odometry();
        }
        
        puclish_odom();
        
    }
    else
    {
        encoder_first_ = true;
    }
    header_pre_ = header_;
    tf_broadcast();

}

void LocalizationClass::odometry()
{
    static double time_pre = 0;
    if (time_pre != 0)
    {
        double vel = encoder_value_.linear.x;
        double ang = encoder_value_.angular.z;
        double dt = header_.stamp.toSec() - time_pre;

        double roll, pitch, theta;
        tf2::Quaternion quat;
        tf2::convert(odom_.pose.pose.orientation, quat);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, theta);

        theta += ang*dt;
        odom_.pose.pose.position.x += vel*cos(theta)*dt;
        odom_.pose.pose.position.y += vel*sin(theta)*dt;
        
        geometry_msgs::Quaternion quat_msg;
        quat.setRPY(0, 0, theta);
        tf2::convert(quat, quat_msg);

        odom_.pose.pose.orientation = quat_msg;
        odom_.twist.twist = encoder_value_;
    }
    time_pre = header_.stamp.toSec();
}

double LocalizationClass::draw_gaussian(double mu, double sigma)
{
    std::default_random_engine engine(seed_gen_());
    std::normal_distribution<> dist(mu, sigma);
    return dist(engine);
}

void LocalizationClass::reset_particle()
{
    for (int i = 0; i < particle_num_; i++)
    {
        particles_.markers[i].pose.position.x = 0;
        particles_.markers[i].pose.position.y = 0;
        particles_.markers[i].pose.position.z = 0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);
        tf2::convert(quat, particles_.markers[i].pose.orientation);

        weights_[i] = 1/particle_num_;
    }
}

void LocalizationClass::create_particle()
{
    //ROS_INFO("create particle");

    double vel = encoder_value_.linear.x;
    double ang = encoder_value_.angular.z;
    double dt = header_.stamp.toSec() - header_pre_.stamp.toSec();
    double unit_vel = sqrt(abs(vel)/dt);
    double unit_ang = sqrt(abs(ang)/dt);
    double maximum_likefood = 0;

    for (int i = 0; i < particle_num_; i++)
    {
        particles_.markers[i].header = header_;
        particles_.markers[i].ns = "Particle_display";
        particles_.markers[i].id = i;
        particles_.markers[i].lifetime = ros::Duration();

        particles_.markers[i].type = visualization_msgs::Marker::ARROW;
        particles_.markers[i].action = visualization_msgs::Marker::ADD;
        
        double delta_vv = draw_gaussian(0.0, COVARIANCE_VV);
        double delta_vomega = draw_gaussian(0.0, COVARIANCE_VOMEGA);
        double delta_omegaomega = draw_gaussian(0.0, COVARIANCE_OMEGAOMEGA);
        
        double vel_dash = vel + (delta_vv*unit_vel) + (delta_vomega*unit_ang);
        double ang_dash = ang + (delta_vomega*unit_vel) + (delta_omegaomega*unit_ang);
        
        double roll, pitch, yaw;
        tf2::Quaternion quat;
        tf2::convert(particles_.markers[i].pose.orientation, quat);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        double theta = yaw + ang_dash*dt;
        double x = particles_.markers[i].pose.position.x + vel_dash*dt*cos(theta);
        double y = particles_.markers[i].pose.position.y + vel_dash*dt*sin(theta);

        //ROS_INFO("vel_dash= %f, ang_dash= %f",vel_dash,ang_dash);

        particles_.markers[i].pose.position.x = x;
        particles_.markers[i].pose.position.y = y;
        particles_.markers[i].pose.position.z = 0;

        quat.setRPY(0, 0, theta);
        tf2::convert(quat, particles_.markers[i].pose.orientation);
        
        //std::cout<< particles_.markers[i].pose <<std::endl;

        double likefood = 0;
        if (local_map_.data.size() > 0)
        {
            local_map_.info.origin.position = particles_.markers[i].pose.position;
            likefood = match_rate(local_map_, world_map_);
            //std::cout<< i <<std::endl;
            if (likefood > maximum_likefood)
            {
                maximum_likefood = likefood;
                maximum_likefood_particle_id_ = i;
            }
        }
        weights_[i] = likefood;
        
        particles_.markers[i].scale.x = likefood + 0.05;
        particles_.markers[i].scale.y = 0.05;
        particles_.markers[i].scale.z = 0.05;

        particles_.markers[i].color.a = 1;

        particles_.markers[i].color.r = 1;
        particles_.markers[i].color.g = 1;
        particles_.markers[i].color.b = 0;
    }

    pub_particle_.publish(particles_);
}

void LocalizationClass::resampling()
{
    ROS_INFO("resampling");
    
    visualization_msgs::MarkerArray particles_new = particles_;

    double W = 0;
    std::vector<double> add_up(particle_num_);
    for (int i = 0; i < particle_num_; i++)
    {
        W += weights_[i];
        add_up[i] = W;
    }

    double increment = W/particle_num_;

    std::default_random_engine engine(seed_gen_());
    std::uniform_real_distribution<> dist(0, increment);
    double r = dist(engine);

    double sum = r;
    int i = 1;
    int idx_new = 0;
    double min = 0;
    while (i < particle_num_)
    {
        if (sum >= min && sum <= add_up[i])
        {
            particles_new.markers[idx_new].pose = particles_.markers[i].pose;
            particles_new.markers[idx_new].scale = particles_.markers[i].scale;
            idx_new++;
            sum += increment;
        }
        else if (sum > add_up[i])
        {
            i++;
            min = add_up[i-1];
        }
    }
    particles_ = particles_new;
}

void LocalizationClass::puclish_odom()
{
    pub_odom_.publish(odom_);
    tf_broadcast();
}

void LocalizationClass::tf_broadcast()
{
    geometry_msgs::TransformStamped tf_map2robot;

    std::string ns = ros::this_node::getNamespace();

    tf_map2robot.header = odom_.header;
    tf_map2robot.child_frame_id = ns;
    tf_map2robot.transform.translation.x = odom_.pose.pose.position.x;
    tf_map2robot.transform.translation.y = odom_.pose.pose.position.y;
    tf_map2robot.transform.translation.z = odom_.pose.pose.position.z;
    tf_map2robot.transform.rotation = odom_.pose.pose.orientation;
    broadcaster_.sendTransform(tf_map2robot);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped tf_robot2lidar;
    tf_robot2lidar.header = odom_.header;
    tf_robot2lidar.header.frame_id = ns;
    tf_robot2lidar.child_frame_id = "/lidar";
    tf_robot2lidar.transform.translation.x = 0.0;
    tf_robot2lidar.transform.translation.y = 0.0;
    tf_robot2lidar.transform.translation.z = 0.1;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tf_robot2lidar.transform.rotation.x = q.x();
    tf_robot2lidar.transform.rotation.y = q.y();
    tf_robot2lidar.transform.rotation.z = q.z();
    tf_robot2lidar.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(tf_robot2lidar);

    geometry_msgs::TransformStamped tf_robot2stereo_camera;
    tf_robot2stereo_camera.header = odom_.header;
    tf_robot2stereo_camera.header.frame_id = ns;
    tf_robot2stereo_camera.child_frame_id = "/stereo_camera";
    tf_robot2stereo_camera.transform.translation.x = 0.1;
    tf_robot2stereo_camera.transform.translation.y = 0.0;
    tf_robot2stereo_camera.transform.translation.z = 0.3;
    q.setRPY(0, 0, 0);
    tf_robot2stereo_camera.transform.rotation.x = q.x();
    tf_robot2stereo_camera.transform.rotation.y = q.y();
    tf_robot2stereo_camera.transform.rotation.z = q.z();
    tf_robot2stereo_camera.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(tf_robot2stereo_camera);
}