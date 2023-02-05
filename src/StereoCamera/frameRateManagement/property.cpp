#include<potbot/frameRateManagement.h>

void managementClass::setFromLaunchfile(){
    
    ros::NodeHandle n("~");
    n.getParam("frameRate",frameRate);
}

float& managementClass::getFrameRate(){
    return frameRate;
}

bool& managementClass::getDuplicationFalg(){
    return dupImageFlag;
}
