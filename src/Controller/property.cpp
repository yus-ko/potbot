#include<potbot/Controller.h>

void ControllerClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("ROBOT_NAME",ROBOT_NAME);
    n.getParam("IS_SIMULATOR",IS_SIMULATOR);
    n.getParam("PATH_PLANNING_FILE",PATH_PLANNING_FILE);
}