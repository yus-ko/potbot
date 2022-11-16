#include<autonomous_mobile_robot_2022/tf.h>

TFClass::TFClass()
{

	//ros::Duration(5).sleep();

	setLaunchParam();	// lanchファイルの読み込み

	sub_scan=nhSub.subscribe("/scan",1,&TFClass::scan_callback,this);
	sub_encoder = nhSub.subscribe("/amcl_pose", 1, &TFClass::pwcs_callback, this);

	
}
TFClass::~TFClass(){
}
