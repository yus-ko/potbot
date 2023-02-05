#include<autonomous_mobile_robot_2022/convCamData.h>

void convCamDataClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //カメラパラメータ
    n.getParam("camera/focus",f);
    n.getParam("camera/cameraHeight",camHeight);
    //マップパラメータ
    n.getParam("localMap/width/float",mapW);
    n.getParam("localMap/height/float",mapH);
    n.getParam("localMap/resolution",mapR);
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]

}