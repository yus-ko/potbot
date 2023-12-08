#include<potbot_rviz/rviz.h>

void rvizClass::PotentialValue_callback(const potbot_msgs::PotentialValue& msg)
{
    PV = msg;
    addMarker();
    publishMarker();
}

void rvizClass::odom_callback(const nav_msgs::Odometry& msg)
{
    odom = msg;
    CreateTraj();
    publishTraj();
}

void rvizClass::manage()
{
    
}

bool is_equal(geometry_msgs::Vector3 A,geometry_msgs::Vector3 B)
{
    if (A.x == B.x && A.y == B.y && A.z == B.z)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void rvizClass::addMarker()
{
    int PV_size = PV.potential_value.size();
    if (PV_size > 0)
    {
        marker_array.markers.resize(PV_size);
        
        for (int i = 0; i < PV_size; i++)
        {
            marker_array.markers[i].header.frame_id = "/odom";
            marker_array.markers[i].header.stamp = ros::Time::now();
            marker_array.markers[i].ns = "PotentialValue_display";
            marker_array.markers[i].id = i;
            marker_array.markers[i].lifetime = ros::Duration();

            marker_array.markers[i].type = visualization_msgs::Marker::SPHERE;
            marker_array.markers[i].action = visualization_msgs::Marker::ADD;

            marker_array.markers[i].pose.position.x = i/PV.cols * PV.x_increment + PV.x_min;
            marker_array.markers[i].pose.position.y = i%PV.cols * PV.y_increment + PV.y_min;
            marker_array.markers[i].pose.position.z = PV.potential_value[i]/POTENTIAL_VALUE_PLOT_SHRINK_SCALE;
            if (marker_array.markers[i].pose.position.z > POTENTIAL_VALUE_PLOT_LIMIT) marker_array.markers[i].pose.position.z = POTENTIAL_VALUE_PLOT_LIMIT;

            marker_array.markers[i].pose.orientation.x = 0.0;
            marker_array.markers[i].pose.orientation.y = 0.0;
            marker_array.markers[i].pose.orientation.z = 0.0;
            marker_array.markers[i].pose.orientation.w = 1.0;

            marker_array.markers[i].color.a = 1;
            
            bool is_path = false;
            int path_size = PV.path_plan.size();
            for (int idx=0; idx<path_size; idx++)
            {
                if (i == PV.path_plan[idx])
                {
                    is_path = true;
                    break;
                }
            }

            if (i == PV.robot_position)
            {
                marker_array.markers[i].scale.x = 0.1;
                marker_array.markers[i].scale.y = 0.1;
                marker_array.markers[i].scale.z = 0.1;

                marker_array.markers[i].color.r = 1;
                marker_array.markers[i].color.g = 0;
                marker_array.markers[i].color.b = 0;
            }
            else if (i == PV.min_potential_position)
            {
                marker_array.markers[i].scale.x = 0.1;
                marker_array.markers[i].scale.y = 0.1;
                marker_array.markers[i].scale.z = 0.1;

                marker_array.markers[i].color.r = 0;
                marker_array.markers[i].color.g = 0;
                marker_array.markers[i].color.b = 1;
            }
            else if (is_path)
            {
                marker_array.markers[i].scale.x = 0.1;
                marker_array.markers[i].scale.y = 0.1;
                marker_array.markers[i].scale.z = 0.1;

                marker_array.markers[i].color.r = 0;
                marker_array.markers[i].color.g = 1;
                marker_array.markers[i].color.b = 0;
            }
            else
            {
                marker_array.markers[i].scale.x = 0.05;
                marker_array.markers[i].scale.y = 0.05;
                marker_array.markers[i].scale.z = 0.05;

                marker_array.markers[i].color.r = 1;
                marker_array.markers[i].color.g = 1;
                marker_array.markers[i].color.b = 1;
            }
           

        }

        

        
    }
}

void rvizClass::CreateTraj()
{
    ros::Time now = ros::Time::now();
    robot_traj_.header = odom.header;
    robot_traj_.header.frame_id = "/map";

    int size = 0;
    if (now.toSec() < CreatePath_time_pre.toSec() + 1) size = robot_traj_.poses.size(); 
    robot_traj_.poses.resize(size+1);
    robot_traj_.poses[size].header = odom.header;
    robot_traj_.poses[size].pose = odom.pose.pose;

    CreatePath_time_pre = now;
}

void rvizClass::publishMarker()
{
    pub_marker.publish(marker_array);
}

void rvizClass::publishTraj()
{
    pub_path.publish(robot_traj_);
}