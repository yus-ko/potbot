#include<potbot_controller/Controller.h>

void ControllerClass::__get_param(){
    
    ros::NodeHandle n("~");
    //n.getParam("",);
    n.getParam("PUBLISH_COMMAND",       PUBLISH_COMMAND);
    n.getParam("PATH_TRACKING_MARGIN",  PATH_TRACKING_MARGIN);
    n.getParam("FRAME_ID/GLOBAL",       FRAME_ID_GLOBAL);
    n.getParam("FRAME_ID/ROBOT_BASE",   FRAME_ID_ROBOT_BASE);
    n.getParam("TARGET/POSITION/X",     TARGET_POSITION_X);
    n.getParam("TARGET/POSITION/Y",     TARGET_POSITION_Y);
    n.getParam("TARGET/POSITION/YAW",   TARGET_POSITION_YAW);
    n.getParam("MAX_LINEAR_VELOCITY",   MAX_LINEAR_VELOCITY);
    n.getParam("MAX_ANGULAR_VELOCITY",  MAX_ANGULAR_VELOCITY);
    n.getParam("TOPIC/ODOM",            TOPIC_ODOM);
    n.getParam("TOPIC/CMD_VEL",         TOPIC_CMD_VEL);

    goal_.pose.position.x = TARGET_POSITION_X;
    goal_.pose.position.y = TARGET_POSITION_Y;
    goal_.pose.orientation = potbot_lib::utility::get_Quat(0,0,TARGET_POSITION_YAW);
}