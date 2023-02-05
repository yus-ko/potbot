#include<potbot/convCamData.h>

convCamDataClass::convCamDataClass()
	:f(370.985),camHeight(0.5),mapW(6.0),mapH(6.0),mapR(0.05)
	
{
	//subscriber
	sub=nhSub.subscribe("converted_depthImage",1,&convCamDataClass::sensor_callback,this);
	//publisher
    pubConv= nhPub1.advertise<potbot::SensorMapData>("cameraMapData", 1);
	//マスク画像
    pubMask= nhPub2.advertise<potbot::MaskImageData>("maskImageData", 1);
	// lanchファイルの読み込み
	setLaunchParam();
	
	// ステレオカメラSN13612のカメラパラメータ
	// [LEFT_CAM_HD]
	// fx=699.649
	// fy=699.649
	// cx=676.895
	// cy=384.83
	// k1=-0.170365
	// k2=0.0231536
	// p1=0
	// p2=0
	
}
convCamDataClass::~convCamDataClass(){
}