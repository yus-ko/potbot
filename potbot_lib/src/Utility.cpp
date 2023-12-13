#include <potbot_lib/Utility.h>

namespace potbot_lib{
    namespace color{
        std_msgs::ColorRGBA get_msg(int color_id)
        {
            std_msgs::ColorRGBA color;
            if(color_id < 0)
            {
                static std::random_device rd;
                static std::mt19937 gen(rd());
                static std::uniform_real_distribution<> dis(0.0, 1.0);
                color.r = dis(gen);
                color.g = dis(gen);
                color.b = dis(gen);
                color.a = 1;
            }
            else
            {
                int num = color_id%8;
                if(num == potbot_lib::color::RED)
                {
                    color.r = 1; color.g = 0; color.b = 0; color.a = 1;
                }
                else if(num == potbot_lib::color::GREEN)
                {
                    color.r = 0; color.g = 1; color.b = 0; color.a = 1;
                }else if(num == potbot_lib::color::BLUE)
                {
                    color.r = 0; color.g = 0; color.b = 1; color.a = 1;
                }else if(num == potbot_lib::color::YELLOW)
                {
                    color.r = 1; color.g = 1; color.b = 0; color.a = 1;
                }else if(num == potbot_lib::color::LIGHT_BLUE)
                {
                    color.r = 0; color.g = 1; color.b = 1; color.a = 1;
                }else if(num == potbot_lib::color::PURPLE)
                {
                    color.r = 1; color.g = 0; color.b = 1; color.a = 1;
                }else if(num == potbot_lib::color::BLACK)
                {
                    color.r = 0; color.g = 0; color.b = 0; color.a = 1;
                }else if(num == potbot_lib::color::WHITE)
                {
                    color.r = 1; color.g = 1; color.b = 1; color.a = 1;
                }
            }
            
            return color;
        }

        std_msgs::ColorRGBA get_msg(std::string color_name)
        {
            if (color_name == "r" || color_name == "red")
            {
                return get_msg(potbot_lib::color::RED);
            }
            else if(color_name == "g" || color_name == "green")
            {
                return get_msg(potbot_lib::color::GREEN);
            }
            else if(color_name == "b" || color_name == "blue")
            {
                return get_msg(potbot_lib::color::BLUE);
            }
            else if(color_name == "y" || color_name == "yellow")
            {
                return get_msg(potbot_lib::color::YELLOW);
            }
            else if(color_name == "lb" || color_name == "light_blue")
            {
                return get_msg(potbot_lib::color::LIGHT_BLUE);
            }
            else if(color_name == "p" || color_name == "purple")
            {
                return get_msg(potbot_lib::color::PURPLE);
            }
            else if(color_name == "k" || color_name == "black")
            {
                return get_msg(potbot_lib::color::BLACK);
            }
            else if(color_name == "w" || color_name == "white")
            {
                return get_msg(potbot_lib::color::WHITE);
            }
            else if(color_name == "random")
            {
                return get_msg(-1);
            }
            else
            {
                return get_msg(potbot_lib::color::RED);
            }
        }
    }

    namespace utility{

        void get_RPY(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw)
        {
            tf2::Quaternion quat;
            tf2::convert(orientation, quat);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        }

        geometry_msgs::Quaternion get_Quat(double roll, double pitch, double yaw)
        {
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion orientation;
            tf2::convert(quat, orientation);
            return orientation;
        }

        double get_Yaw(geometry_msgs::Quaternion orientation)
        {
            double roll, pitch, yaw;
            get_RPY(orientation, roll, pitch, yaw);
            return yaw;
        }

        double get_Distance(geometry_msgs::Point position1, geometry_msgs::Point position2)
        {
            return sqrt(pow(position2.x - position1.x,2) + pow(position2.y - position1.y,2));
        }

        double get_Distance(geometry_msgs::Pose position1, geometry_msgs::Pose position2)
        {
            return get_Distance(position1.position, position2.position);
        }

        double get_Distance(geometry_msgs::PoseStamped position1, geometry_msgs::PoseStamped position2)
        {
            return get_Distance(position1.pose.position, position2.pose.position);
        }

        double get_Distance(nav_msgs::Odometry position1, nav_msgs::Odometry position2)
        {
            return get_Distance(position1.pose.pose.position, position2.pose.pose.position);
        }

        void print_Pose(geometry_msgs::Pose pose)
        {
            double r,p,y;
            get_RPY(pose.orientation,r,p,y);
            ROS_INFO("(x,y,z) = (%f, %f, %f) (r,p,y) = (%f, %f, %f)", 
                        pose.position.x, pose.position.y, pose.position.z,
                        r/M_PI*180, p/M_PI*180, y/M_PI*180);
        }

        void print_Pose(geometry_msgs::PoseStamped pose)
        {
            print_Pose(pose.pose);
        }

        void print_Pose(nav_msgs::Odometry pose)
        {
            print_Pose(pose.pose.pose);
        }

        int get_tf(geometry_msgs::PoseStamped pose_in, geometry_msgs::PoseStamped &pose_out, tf2_ros::Buffer &buffer)
        {
            static tf2_ros::TransformListener tf_listener(buffer);
            std::string target_frame = pose_out.header.frame_id;
            std::string source_frame = pose_in.header.frame_id;
            ros::Time transform_time = pose_in.header.stamp;
            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = buffer.lookupTransform(target_frame, source_frame, transform_time);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN_STREAM("get_tf TF2 exception: " << ex.what());
                return FAIL;
            }

            tf2::doTransform(pose_in, pose_out, transformStamped);
            return SUCCESS;
        }

        int get_WorldCoordinate(std::string target_frame, ros::Time time, geometry_msgs::PoseStamped &Wcood, tf2_ros::Buffer &buffer)
        {
            tf2_ros::TransformListener tf_listener(buffer);

            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = buffer.lookupTransform("map", target_frame, time);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN_STREAM("get_WorldCoordinate TF2 exception: " << ex.what());
                return FAIL;
            }
            // ROS_INFO("x: %f, y: %f, z: %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            Wcood.header = transformStamped.header;
            Wcood.pose.position.x = transformStamped.transform.translation.x;
            Wcood.pose.position.y = transformStamped.transform.translation.y;
            Wcood.pose.position.z = transformStamped.transform.translation.z;
            Wcood.pose.orientation.x = transformStamped.transform.rotation.x;
            Wcood.pose.orientation.y = transformStamped.transform.rotation.y;
            Wcood.pose.orientation.z = transformStamped.transform.rotation.z;
            Wcood.pose.orientation.w = transformStamped.transform.rotation.w;
            return SUCCESS;
        }

        geometry_msgs::Point get_MapCoordinate(int index, nav_msgs::MapMetaData info)
        {
            geometry_msgs::Point p;
            p.x = (index % info.width) * info.resolution + info.origin.position.x;
            p.y = (index / info.width) * info.resolution + info.origin.position.y;
            return p;
        }

        int get_MapIndex(double x, double y, nav_msgs::MapMetaData info)
        {

            double xmin = info.origin.position.x;
            double xmax = info.origin.position.x + info.width * info.resolution;

            double ymin = info.origin.position.y;
            double ymax = info.origin.position.y + info.height * info.resolution;

            if (x < xmin || x > xmax || y < ymin || y > ymax)
            {
                return 0;
            }

            double img_x = x - info.origin.position.x;
            double img_y = y - info.origin.position.y;

            int index = int(img_y / info.resolution) * info.width + int(img_x / info.resolution);

            if (index < 0)
            {
                //ROS_INFO("%f, %f, %f, %f",x,y,info.origin.position.x,info.origin.position.y);
                index = 0;
            }

            return index;
            
        }

        int get_PathIndex(nav_msgs::Path path, geometry_msgs::Point position)
        {
            double min_distance = std::numeric_limits<double>::infinity();
            int path_index = 0;
            for (int i = 0; i < path.poses.size(); i++)
            {
                double distance = get_Distance(path.poses[i].pose.position, position);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    path_index = i;
                }
            }
            return path_index;
        }

        int get_PathIndex(nav_msgs::Path path, geometry_msgs::Pose position)
        {
            return get_PathIndex(path, position.position);
        }

        int get_PathIndex(nav_msgs::Path path, geometry_msgs::PoseStamped position)
        {
            return get_PathIndex(path, position.pose.position);
        }

        int get_PathIndex(nav_msgs::Path path, nav_msgs::Odometry position)
        {
            return get_PathIndex(path, position.pose.pose.position);
        }

        double get_PathLength(nav_msgs::Path path)
        {
            double total_length = 0;
            for (int i = 1; i < path.poses.size(); i++)
            {
                total_length += get_Distance(path.poses[i].pose.position, path.poses[i-1].pose.position);
            }
            return total_length;
        }

        void Timer::start(const std::string timer_name, const double time)
        {
            if (time < 0)
            {
                times_[timer_name].begin_time = ros::Time::now().toSec();
            }
            else
            {
                times_[timer_name].begin_time = time;
            }
            times_[timer_name].running = true;
        }

        void Timer::start(const std::vector<std::string> timer_names)
        {
            double time_now = ros::Time::now().toSec();
            for (auto timer_name : timer_names)
            {
                start(timer_name, time_now);
            }
        }

        void Timer::stop(const std::string timer_name, const double time)
        {
            if (!times_[timer_name].running) return;

            if (time < 0)
            {
                times_[timer_name].end_time = ros::Time::now().toSec();
            }
            else
            {
                times_[timer_name].end_time = time;
            }
            times_[timer_name].running = false;
            times_[timer_name].duration = times_[timer_name].end_time - times_[timer_name].begin_time;
        }

        void Timer::stop(const std::vector<std::string> timer_names)
        {
            std::vector<std::string> stop_times = timer_names;
            if(stop_times.empty())
            {
                for (auto it = times_.begin(); it != times_.end(); ++it) 
                {
                    stop_times.push_back(it->first);
                }
            }

            double time_now = ros::Time::now().toSec();
            for (auto timer_name : stop_times)
            {
                stop(timer_name, time_now);
            }
        }

        void Timer::print_time(const std::vector<std::string> timer_names)
        {
            std::vector<std::string> print_times = timer_names;
            if(print_times.empty())
            {
                for (auto it = times_.begin(); it != times_.end(); ++it) 
                {
                    print_times.push_back(it->first);
                }
            }
            
            for(std::string timer_name : print_times)
            {
                std::cout<< timer_name <<": ";
                if (times_[timer_name].running)
                {
                    std::cout<< "running ";
                }
                else
                {
                    std::cout<< times_[timer_name].duration << " [s] ";
                }
            }
            std::cout<<std::endl;
        }

        void Timer::print_time(const std::string timer_name)
        {
            print_time((std::vector<std::string>){timer_name});  
        }

    }

}