#include<ros/ros.h>
#include<tf2_ros/transform_listener.h>
#include<potbot_controller/Controller.h>
#include<potbot_filter/2dscan_clustering.h>
#include<potbot_filter/Filter.h>
#include<potbot_pathplanner/PathPlanning.h>
#include<potbot_pcl/Clustering3D.h>
#include<potbot_localmap/Localmap.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"potbot_nav");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    ControllerClass cc(             buffer, "controller");
    potbot_filter::FilterClass fc;
    scan2dClass s2d(                buffer, "scan2d");
    PathPlanningClass pp(           buffer, "path_planner");
    Clustering3DClass c3d(                  "clustering3d");
    LocalmapClass lm(               buffer, "localmap");
    
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

	return 0;
}