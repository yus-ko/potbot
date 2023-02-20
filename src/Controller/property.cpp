#include<potbot/Controller.h>

void ControllerClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //n.getParam("",);
    n.getParam("ROBOT_NAME",ROBOT_NAME);
    n.getParam("IS_SIMULATOR",IS_SIMULATOR);
    n.getParam("PUBLISH_COMMAND",PUBLISH_COMMAND);
    n.getParam("PATH_TRACKING_MARGIN",PATH_TRACKING_MARGIN);
}