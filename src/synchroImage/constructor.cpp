#include<autonomous_mobile_robot_2022/synchroImage.h>

syncroImageClass::syncroImageClass()
	:rgb_sub(nhSub, "/mynteye/left/image_raw", 1),depth_sub(nhSub, "/mynteye/depth/image_raw", 1)//,sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(10), rgb_sub, depth_sub)
    ,sync(MySyncPolicy(10),rgb_sub, depth_sub),debugType(0)
{
    //subscriber
    sync.registerCallback(boost::bind(&syncroImageClass::callback, this,_1, _2));
    pub= nhPub.advertise<autonomous_mobile_robot_2022::synchronizedImage>("syncronized_image", 1);
	pubDebPcl = nhDeb.advertise<sensor_msgs::PointCloud2>("synchroPoints", 1);
}
syncroImageClass::~syncroImageClass(){

}

