#include<autonomous_mobile_robot_2022/frameRateManagement.h>

managementClass::managementClass()
    :frameRate(17),dupImageFlag(false)
{
    //subscriber
	nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("syncronized_image",1,&managementClass::callback,this);
	//publisher
    //rgb
    pubRgb= nhPub1.advertise<sensor_msgs::Image>("converted_rgbImage", 1);
	//depth
    pubDepth= nhPub2.advertise<sensor_msgs::Image>("converted_depthImage", 1);
    setFromLaunchfile();
}
managementClass::~managementClass(){

}