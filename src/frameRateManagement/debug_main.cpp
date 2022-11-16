#include <autonomous_mobile_robot_2022/frameRateManagement.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_frm");
	
    managementClass mc; //syncro class
    ros::Rate rate(mc.getFrameRate());
    while(ros::ok()){
        mc.subscribeData();
        //debug to desplay times
        mc.delayTime();
        mc.culcPastTime();
        //publish
        mc.publishRgbCamData();
        mc.publishDepthCamData();
        if(mc.getDuplicationFalg()){
            ROS_INFO_STREAM("データがかぶっているよ");
        }
        //sleep
        rate.sleep();
    }

	return 0;
}