#include<potbot/Filter.h>

void FilterClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //n.getParam("",);
    n.getParam("SIGMA_P",SIGMA_P);
    n.getParam("SIGMA_Q",SIGMA_Q);
    n.getParam("SIGMA_R",SIGMA_R);
}