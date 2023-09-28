#include<potbot/PathPlanning.h>
#include<ros/param.h>

void PathPlanningClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //n.getParam("",);
    n.getParam("YAML_FILE",YAML_FILE);
    n.getParam("PATH_PLANNING_METHOD",PATH_PLANNING_METHOD);
    n.getParam("PATH_PLANNING_FILE",PATH_PLANNING_FILE);
    n.getParam("USE_AMCL",USE_AMCL);
    n.getParam("TARGET/POSITION/X",TARGET_POSITION_X);
    n.getParam("TARGET/POSITION/Y",TARGET_POSITION_Y);
    n.getParam("TARGET/POSITION/YAW",TARGET_POSITION_YAW);
    n.getParam("POTENTIAL_FIELD_WIDTH",POTENTIAL_FIELD_WIDTH);
    n.getParam("POTENTIAL_FIELD_DIVDE_X",POTENTIAL_FIELD_DIVDE_X);
    n.getParam("POTENTIAL_FIELD_DIVDE_Y",POTENTIAL_FIELD_DIVDE_Y);
    n.getParam("FRAME_ID/GLOBAL",FRAME_ID_GLOBAL);
    n.getParam("FRAME_ID/ROBOT_BASE",FRAME_ID_ROBOT_BASE);

    goal_.pose.position.x = TARGET_POSITION_X;
    goal_.pose.position.y = TARGET_POSITION_Y;
    goal_.pose.orientation = get_Quat(0,0,TARGET_POSITION_YAW);
}

void PathPlanningClass::__set_Param(){
    ros::NodeHandle n("~");
    int param1;
    std::string param2;
    n.getParam("param1",param1);
    n.getParam("param2",param2);
    ROS_INFO("param1: %d", param1);
    ROS_INFO("param2: %s", param2.c_str());
}