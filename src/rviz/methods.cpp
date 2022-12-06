#include<autonomous_mobile_robot_2022/rviz.h>

void rvizClass::PotentialValue_callback(const autonomous_mobile_robot_2022::PotentialValue& msg)
{
    PV = msg;
    addMarker();
    publishMarker();
}

void rvizClass::odom_callback(const nav_msgs::Odometry& msg)
{
    odom = msg;
    CreatePath();
    publishPath();
}

void rvizClass::PathPlan_callback(const autonomous_mobile_robot_2022::PathPlan& msg)
{
    PP = msg;
    addMarker_pathplan();
    publishMarker_pathplan();
}

void rvizClass::manage()
{
    addMarker();
    publishMarker();
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

void rvizClass::addMarker_pathplan()
{

    int PP_size = PP.data.size();
    int marker_size = marker_array_pp.markers.size();
    int marker_resize = marker_size + PP_size;

    visualization_msgs::MarkerArray marker_array_pp_tmp;
    marker_array_pp_tmp.markers.resize(PP_size);

    int PP_index = 0;
    for (int i = 0; i < PP_size; i++)
    //for (int i = marker_size; i < marker_resize; i++)
    {
        marker_array_pp_tmp.markers[i].header.frame_id = "/odom";
        marker_array_pp_tmp.markers[i].header.stamp = ros::Time::now();
        marker_array_pp_tmp.markers[i].ns = "PathPlan_display";
        marker_array_pp_tmp.markers[i].id = i;
        marker_array_pp_tmp.markers[i].lifetime = ros::Duration();

        marker_array_pp_tmp.markers[i].type = visualization_msgs::Marker::SPHERE;
        marker_array_pp_tmp.markers[i].action = visualization_msgs::Marker::ADD;

        marker_array_pp_tmp.markers[i].pose.position.x = PP.data[PP_index].x;
        marker_array_pp_tmp.markers[i].pose.position.y = PP.data[PP_index].y;
        marker_array_pp_tmp.markers[i].pose.position.z = 0;

        marker_array_pp_tmp.markers[i].scale.x = 0.08;
        marker_array_pp_tmp.markers[i].scale.y = 0.08;
        marker_array_pp_tmp.markers[i].scale.z = 0.08;

        marker_array_pp_tmp.markers[i].pose.orientation.x = 0.0;
        marker_array_pp_tmp.markers[i].pose.orientation.y = 0.0;
        marker_array_pp_tmp.markers[i].pose.orientation.z = 0.0;
        marker_array_pp_tmp.markers[i].pose.orientation.w = 1.0;

        marker_array_pp_tmp.markers[i].color.a = 1;
        marker_array_pp_tmp.markers[i].color.r = 1;
        marker_array_pp_tmp.markers[i].color.g = 1;
        marker_array_pp_tmp.markers[i].color.b = 0;

        PP_index++;

    }
    marker_array_pp = marker_array_pp_tmp;
    //std::cout<< "(x,y) = (" << marker_array_pp.markers[0].pose.position.x << "," << marker_array_pp.markers[0].pose.position.y << ")" <<std::endl;

}

void rvizClass::CreatePath()
{
    ros::Time now = ros::Time::now();

    robot_path.header.frame_id = "/odom";
    robot_path.header.stamp = now;

    int size = 0;
    if (now.toSec() < CreatePath_time_pre.toSec() + 1) size = robot_path.poses.size(); 
    robot_path.poses.resize(++size);
    robot_path.poses[size-1].pose = odom.pose.pose;

    CreatePath_time_pre = now;
}

void rvizClass::publishMarker()
{
    pub_marker.publish(marker_array);
}

void rvizClass::publishMarker_pathplan()
{
    pub_marker_pp.publish(marker_array_pp);
}

void rvizClass::publishPath()
{
    pub_path.publish(robot_path);
}