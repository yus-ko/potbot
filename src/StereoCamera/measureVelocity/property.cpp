#include<potbot/measurementVelocity.h>

void measurementVelocity::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("measurementVelocity/rqt_reconfigure",rqt_reconfigure);
    //マッチングパラメータ
    n.getParam("measurementVelocity/weight/image", weightImage);
    n.getParam("measurementVelocity/weight/size", weightSize);
    n.getParam("measurementVelocity/weight/gravity", weightGravity);
    //追跡閾値
    n.getParam("measurementVelocity/trackThreshold", trackThreshold);
    //デバッグ
    n.getParam("measurementVelocity/debugType",debugType);
    n.getParam("measurementVelocity/timeRange",timeRange);
    n.getParam("measurementVelocity/timeInteval",timeInteval);

}
// void measurementVelocity::configCallback(potbot::measurementVelocityConfig &config, uint32_t level) {
// 	ROS_INFO("Reconfigure Request: %f %f %f %d %d %f %f", 
// 		config.weightImage, config.weightSize, config.weightGravity, 
//         config.trackThreshold,
//         config.debugType, config.timeRange, config.timeInteval
// 		// config.str_param.c_str(), 
// 		// config.bool_param?"True":"False", 
// 		// config.size
// 		);
//     weightImage = config.weightImage;
//     weightSize = config.weightSize;
//     weightGravity = config.weightGravity;
//     trackThreshold = config.trackThreshold;
//     debugType = config.debugType;
//     timeRange = config.timeRange;
//     timeInteval = config.timeInteval;
// }
