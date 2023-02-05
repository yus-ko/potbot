#include<potbot/velocityEstimation.h>

velocityEstimation::velocityEstimation()
{
	//ROS_INFO("subscriber define");
	//subscriber
	// nhSub1.setCallbackQueue(&queue1);
	sub=nhSub1.subscribe("measurementVelocityCluster",1,&velocityEstimation::cluster_callback,this);
	//publisher
	//ROS_INFO("publisher define");
    pub= nhPub.advertise<potbot::ClassificationVelocityData>("classificationDataEstimateVelocity", 1);

	//デバッグ用
	pubDebPcl= nhDeb.advertise<sensor_msgs::PointCloud2>("debugEstimatedVelocity", 1);
	pubDebMarker= nhDeb.advertise<visualization_msgs::MarkerArray>("estimatedVelocityMarker", 1);


	//--calman filter parameter dynamicReconfigureに追加予定
	//resize
	//ROS_INFO("resize");
	sig_ut = Eigen::MatrixXd::Zero(2,2);
	// del_t = Eigen::MatrixXd::Zero(4,4);
	// sig_x0 = Eigen::MatrixXd::Zero(4,4);
	// sig_wk = Eigen::MatrixXd::Zero(4,4);
	// I = Eigen::MatrixXd::Identity(4,4);
	// 変更
	del_t = Eigen::MatrixXd::Zero(5,5);
	sig_x0 = Eigen::MatrixXd::Zero(5,5);
	sig_wk = Eigen::MatrixXd::Zero(5,5);
	I = Eigen::MatrixXd::Identity(5,5);
	//initialize
	//ROS_INFO("init");
	//delta Q
	//--観測誤差共分散
	del_t(0,0)=0.05*0.05;//0.01;//x
	del_t(1,1)=0.05*0.05;//0.01;//z
	del_t(2,2)=0.02;//0.09;//0.04;//0.25;//vx
	del_t(3,3)=0.02;//0.09;//0.04;//0.25;//vz
	del_t(4,4)=0;//theta変更
	//--
	del_t(0,2)=0;//del_t(0,0)*del_t(2,2)/2;
	del_t(1,3)=0;//del_t(1,1)*del_t(3,3)/2;
	del_t(2,0)=0;//del_t(0,2);//del_t(0,0)/(dt*dt);
	del_t(3,1)=0;//del_t(1,3);//del_t(1,1)/(dt*dt);
	//sigma
	//--推定誤差共分散
	sig_x0(0,0)=0.01;//0.04;//x
	sig_x0(1,1)=0.01;//0.04;//z
	sig_x0(2,2)=0.09;//vx
	sig_x0(3,3)=0.09;//vz
	sig_x0(4,4)=0;//theta 変更
	//--
	sig_x0(0,2)=0;//sig_x0(2,2)*(0.2*0.2);
	sig_x0(2,0)=0;//sig_x0(0,2);//sig_x0(0,0)/(0.2*0.2);
	sig_x0(1,3)=0;//sig_x0(3,3)*(0.2*0.2);
	sig_x0(3,1)=0;//sig_x0(1,3);//sig_x0(1,1)/(0.2*0.2);
	//sigma
	//--モデル誤差共分散
	sig_wk(0,0)=0.05*0.05;//x
	sig_wk(1,1)=0.05*0.05;//z
	sig_wk(2,2)=0.02;//0.01;//vx
	sig_wk(3,3)=0.02;//0.01;//vz
	sig_wk(4,4)=0;//theta 変更
	//--
	sig_wk(0,2)=0;//sig_wk(0,0)*sig_wk(2,2)/2;
	sig_wk(1,3)=0;//sig_wk(3,3)*ig_wk(1,1)/2;
	sig_wk(2,0)=0;//sig_wk(0,2);//sig_wk(0,0)/(dt*dt);
	sig_wk(3,1)=0;//sig_wk(1,3);//sig_wk(1,1)/(dt*dt);
	//--
    trackThreshold = 1;
	sizeMinThreshold = 0;
	sizeMaxThreshold = 1;
	velSigmaThreshold = 1;
	velMinThreshold = 0.1;
	velMaxThreshold = 1.5;
	//average filter
	filterN = 1;

	//launchファイルからパラメータの読み込み
	//ROS_INFO("setLaunchParam");
	setLaunchParam();
	//
	//rqt_reconfigure
	// f = boost::bind(&velocityEstimation::configCallback, this, _1, _2);
	// server.setCallback(f);
	ROS_INFO("ready");
}
velocityEstimation::~velocityEstimation(){
}