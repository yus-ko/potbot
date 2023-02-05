#include<potbot/velocityEstimation.h>

void velocityEstimation::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //カルマンフィルタパラメータ
	//--観測誤差共分散
    n.getParam("observationDelta/1/1",del_t(0,0));
    n.getParam("observationDelta/2/2",del_t(1,1));
    n.getParam("observationDelta/3/3",del_t(2,2));
    n.getParam("observationDelta/4/4",del_t(3,3));
    n.getParam("observationDelta/5/5",del_t(4,4));
    n.getParam("observationDelta/1/3",del_t(0,2));
    n.getParam("observationDelta/3/1",del_t(2,0));
    n.getParam("observationDelta/2/4",del_t(1,3));
    n.getParam("observationDelta/4/2",del_t(3,1));
	//--モデル誤差共分散
    n.getParam("predictionSigma/1/1",sig_wk(0,0));
    n.getParam("predictionSigma/2/2",sig_wk(1,1));
    n.getParam("predictionSigma/3/3",sig_wk(2,2));
    n.getParam("predictionSigma/4/4",sig_wk(3,3));
    n.getParam("predictionSigma/5/5",sig_wk(4,4));
    n.getParam("predictionSigma/1/3",sig_wk(0,2));
    n.getParam("predictionSigma/3/1",sig_wk(2,0));
    n.getParam("predictionSigma/2/4",sig_wk(1,3));
    n.getParam("predictionSigma/4/2",sig_wk(3,1));
	//--推定共分散の初期値
    n.getParam("estimationSigma/1/1",sig_x0(0,0));
    n.getParam("estimationSigma/2/2",sig_x0(1,1));
    n.getParam("estimationSigma/3/3",sig_x0(2,2));
    n.getParam("estimationSigma/4/4",sig_x0(3,3));
    n.getParam("estimationSigma/5/5",sig_x0(4,4));
    n.getParam("estimationSigma/1/3",sig_x0(0,2));
    n.getParam("estimationSigma/3/1",sig_x0(2,0));
    n.getParam("estimationSigma/2/4",sig_x0(1,3));
    n.getParam("estimationSigma/4/2",sig_x0(3,1));
    //追跡閾値
    n.getParam("measurementVelocity/trackThreshold", trackThreshold);
	n.getParam("measurementVelocity/sizeMinThreshold", sizeMinThreshold);
	n.getParam("measurementVelocity/sizeMaxThreshold", sizeMaxThreshold);
	n.getParam("measurementVelocity/velSigmaThreshold", velSigmaThreshold);
	n.getParam("measurementVelocity/velMinThreshold", velMinThreshold);
	n.getParam("measurementVelocity/velMaxThreshold", velMaxThreshold);
    //平均フィルタ
	n.getParam("measurementVelocity/filterN", filterN);
	//デバッグ
    n.getParam("measurementVelocity/debugType",debugType);
    n.getParam("measurementVelocity/timeRange",timeRange);
    n.getParam("measurementVelocity/timeInteval",timeInteval);
}
// void velocityEstimation::configCallback(potbot::velocityEstimationConfig &config, uint32_t level) {
// 	// ROS_INFO("Reconfigure Request: %d %f %f %d", 
// 	// 	config.windowDivisionDegree, config.windowHeight,
// 	// 	config.windowWidth,config.windowMinPts
// 	// 	// config.str_param.c_str(), 
// 	// 	// config.bool_param?"True":"False", 
// 	// 	// config.size
// 	// 	);
// 	//観測誤差共分散
// 	del_t(0,0) = config.observationDelta11;
// 	del_t(1,1) = config.observationDelta22;
// 	del_t(2,2) = config.observationDelta33;
// 	del_t(3,3) = config.observationDelta44;
// 	del_t(4,4) = config.observationDelta55;
// 	del_t(0,2) = config.observationDelta13;
// 	del_t(1,3) = config.observationDelta31;
// 	del_t(2,0) = config.observationDelta24;
// 	del_t(3,1) = config.observationDelta42;
// 	//モデル誤差共分散
// 	sig_wk(0,0) = config.predictionSigma11;
// 	sig_wk(1,1) = config.predictionSigma22;
// 	sig_wk(2,2) = config.predictionSigma33;
// 	sig_wk(3,3) = config.predictionSigma44;
// 	sig_wk(4,4) = config.predictionSigma55;
// 	sig_wk(0,2) = config.predictionSigma13;
// 	sig_wk(1,3) = config.predictionSigma31;
// 	sig_wk(2,0) = config.predictionSigma24;
// 	sig_wk(3,1) = config.predictionSigma42;
// 	//推定共分散の初期値
// 	sig_x0(0,0) = config.estimationSigma11;
// 	sig_x0(1,1) = config.estimationSigma22;
// 	sig_x0(2,2) = config.estimationSigma33;
// 	sig_x0(3,3) = config.estimationSigma44;
// 	sig_x0(4,4) = config.estimationSigma55;
// 	sig_x0(0,2) = config.estimationSigma13;
// 	sig_x0(1,3) = config.estimationSigma31;
// 	sig_x0(2,0) = config.estimationSigma24;
// 	sig_x0(3,1) = config.estimationSigma42;
// 	//移動障害物判断パラメータ
//     trackThreshold = config.trackThreshold;
// 	sizeMinThreshold = config.sizeMinThreshold;
// 	sizeMaxThreshold = config.sizeMaxThreshold;
// 	velSigmaThreshold = config.velSigmaThreshold;
// 	velMinThreshold = config.velMinThreshold;
// 	velMaxThreshold = config.velMaxThreshold;
// 	//average filter
// 	filterN = config.filterN;
// 	//デバッグ
//     debugType = config.debugType;
//     timeRange = config.timeRange;
//     timeInteval = config.timeInteval;
// }