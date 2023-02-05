#include<potbot/imageMatching.h>

imageMatchingClass::imageMatchingClass()
	:imgCurOnce(false),imgPreOnce(false),debugType(1),mapW(8),mapH(8),mapR(0.05),
	trackThreshold(2),nh(8),nw(16),maxDetectPoint(20),maxPoint(10),ws(13),rqt_reconfigure(true)
{

	//subscriber
	// nhSub1.setCallbackQueue(&queue1);
	subImg=nhSub1.subscribe("converted_rgbImage",1,&imageMatchingClass::image_callback,this);
	// nhSub2.setCallbackQueue(&queue2);
	subMskImg=nhSub2.subscribe("maskImageData",1,&imageMatchingClass::maskImage_callback,this);
	//publisher
    pubMatch= nhPub.advertise<potbot::ImageMatchingData>("imageMatchingData", 1);
	pubDeb= nhDeb.advertise<sensor_msgs::Image>("debugImageData", 1);
	pubDebPcl = nhDeb.advertise<sensor_msgs::PointCloud2>("groundDeletePoints", 1);
	
	//launchファイルから読み込み
	setLaunchParam();
	
	//rqt_reconfigure
//     if(rqt_reconfigure){
// 		f = boost::bind(&imageMatchingClass::configCallback, this, _1, _2);
// 		server.setCallback(f);
// 	}
}
imageMatchingClass::~imageMatchingClass(){
	
}