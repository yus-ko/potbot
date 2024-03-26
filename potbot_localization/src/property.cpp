#include<potbot_localization/Localization.h>

void LocalizationClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("LOCALIZATION_METHOD",   LOCALIZATION_METHOD);
    n.getParam("IS_SIMULATOR",          IS_SIMULATOR);
    n.getParam("COVARIANCE_VV",         COVARIANCE_VV);
    n.getParam("COVARIANCE_VOMEGA",     COVARIANCE_VOMEGA);
    n.getParam("COVARIANCE_OMEGAOMEGA", COVARIANCE_OMEGAOMEGA);
    n.getParam("INITIAL_POSE_X",        INITIAL_POSE_X);
    n.getParam("INITIAL_POSE_Y",        INITIAL_POSE_Y);
    n.getParam("INITIAL_POSE_THETA",    INITIAL_POSE_THETA);
    // n.getParam("FRAME_ID/GLOBAL",FRAME_ID_GLOBAL);
    n.getParam("FRAME_ID/ROBOT_BASE",FRAME_ID_ROBOT_BASE);
    // n.getParam("FRAME_ID/LIDAR",FRAME_ID_LIDAR);
    n.getParam("TOPIC/SCAN",            TOPIC_SCAN);
    n.getParam("TOPIC/ODOM",            TOPIC_ODOM);
}