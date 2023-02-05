#include<autonomous_mobile_robot_2022/synchroImage.h>

void syncroImageClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("debugType",debugType);
}

