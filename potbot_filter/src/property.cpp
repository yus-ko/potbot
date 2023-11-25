#include<potbot_filter/Filter.h>

void FilterClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //n.getParam("",);
    n.getParam("SIGMA_P",               SIGMA_P);
    n.getParam("SIGMA_Q",               SIGMA_Q);
    n.getParam("SIGMA_R",               SIGMA_R);
    n.getParam("FRAME_ID/GLOBAL",       FRAME_ID_GLOBAL);
    n.getParam("FRAME_ID/ROBOT_BASE",   FRAME_ID_ROBOT_BASE);
    n.getParam("FRAME_ID/LIDAR",        FRAME_ID_LIDAR);
    n.getParam("TOPIC/SCAN",            TOPIC_SCAN);
    n.getParam("TOPIC/ODOM",            TOPIC_ODOM);
}