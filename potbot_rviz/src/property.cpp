#include<potbot_rviz/rviz.h>

void rvizClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("POTENTIAL_VALUE_PLOT_LIMIT",POTENTIAL_VALUE_PLOT_LIMIT);
    n.getParam("POTENTIAL_VALUE_PLOT_SHRINK_SCALE",POTENTIAL_VALUE_PLOT_SHRINK_SCALE);
    //n.getParam("IS_SIMULATOR",IS_SIMULATOR);

}