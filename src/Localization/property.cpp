#include<potbot/Localization.h>

void LocalizationClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("ROBOT_NAME",ROBOT_NAME);
    n.getParam("LOCALIZATION_METHOD",LOCALIZATION_METHOD);
    n.getParam("IS_SIMULATOR",IS_SIMULATOR);
    n.getParam("COVARIANCE_VV",COVARIANCE_VV);
    n.getParam("COVARIANCE_VOMEGA",COVARIANCE_VOMEGA);
    n.getParam("COVARIANCE_OMEGAOMEGA",COVARIANCE_OMEGAOMEGA);
    n.getParam("INITIAL_POSE_X",INITIAL_POSE_X);
    n.getParam("INITIAL_POSE_Y",INITIAL_POSE_Y);
    n.getParam("INITIAL_POSE_THETA",INITIAL_POSE_THETA);
}