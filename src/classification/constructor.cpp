#include<autonomous_mobile_robot_2022/classification.h>

classificationClass::classificationClass()
	:minCamDeg(-60),maxCamDeg(60),winDivDeg(3),winDivNum(9)
	,mapWidth(8.0),mapHeight(8.0),mapRes(0.05),mapWidthInt(160),mapHeightInt(160)
	,widthWin(0.4),heightWin(0.1),minPts(100),rqt_reconfigure(true)
{
	//subscriber
	// nhSub1.setCallbackQueue(&queue1);
	subCam=nhSub1.subscribe("cameraMapData",1,&classificationClass::cameraMap_callback,this);
	// nhSub2.setCallbackQueue(&queue2);
	subLRF=nhSub2.subscribe("laserMapData",1,&classificationClass::laserMap_callback,this);
	//publisher
    pub= nhPub.advertise<autonomous_mobile_robot_2022::ClassificationData>("classificationData", 1);

	//launchファイルからパラメータの読み込み
	setLaunchParam();
	//デバッグ用
	//publisher
    pubDeb= nhDeb.advertise<sensor_msgs::Image>("windowImage", 1);
    pubDebPcl= nhDebPcl.advertise<sensor_msgs::PointCloud2>("visualizedCluster", 1);
    pubDebGp= nhDebGp.advertise<sensor_msgs::PointCloud2>("visualizedGravityPoints", 1);
	
	//rqt_reconfigure
    // if(rqt_reconfigure){
	// 	f = boost::bind(&classificationClass::configCallback, this, _1, _2);
	// 	server.setCallback(f);
	// }
}
classificationClass::~classificationClass(){
	winIndex.clear();
}