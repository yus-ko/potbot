#include <potbot_lib/Utility.h>

namespace potbot_lib{
    namespace utility{

        void getRPY(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw)
        {
            tf2::Quaternion quat;
            tf2::convert(orientation, quat);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        }

        void getQuat(double roll, double pitch, double yaw, geometry_msgs::Quaternion &orientation)
        {
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            tf2::convert(quat, orientation);
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
            getRPY(orientation, roll, pitch, yaw);
            return yaw;
        }

        double get_Distance(geometry_msgs::Point position1, geometry_msgs::Point position2)
        {
            return sqrt(pow(position2.x - position1.x,2) + pow(position2.y - position1.y,2));
        }

        void print_Pose(geometry_msgs::Pose pose)
        {
            double r,p,y;
            getRPY(pose.orientation,r,p,y);
            ROS_INFO("(x,y,z) = (%f, %f, %f) (r,p,y) = (%f, %f, %f)", 
                        pose.position.x, pose.position.y, pose.position.z,
                        r/M_PI*180, p/M_PI*180, y/M_PI*180);
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

        geometry_msgs::Point get_coordinate(int index, nav_msgs::MapMetaData info)
        {
            geometry_msgs::Point p;
            p.x = (index % info.width) * info.resolution + info.origin.position.x;
            p.y = (index / info.width) * info.resolution + info.origin.position.y;
            return p;
        }

        int get_index(double x, double y, nav_msgs::MapMetaData info)
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
    }
}