#include<potbot_filter/Filter.h>

void FilterClass::__get_param(){
    
    ros::NodeHandle n("~");
    //n.getParam("",);
    n.getParam("SIGMA/P",               SIGMA_P);
    n.getParam("SIGMA/Q",               SIGMA_Q);
    n.getParam("SIGMA/R",               SIGMA_R);
}