#include<autonomous_mobile_robot_2022/synchroImage.h>


//subscribe
void syncroImageClass::callback(const sensor_msgs::Image::ConstPtr& rgbMsg,const sensor_msgs::Image::ConstPtr& depthMsg){
    rgbImage = *rgbMsg;
    depthImage = *depthMsg;
    // std::cout<<ros::Duration(depthImage.header.stamp-rgbImage.header.stamp).toSec()<<"\n";
    conbineImage();
    publishRgbCamData();
    //debug
    // std::cout<<"showSynchroPCL "<<std::endl;
    debug();
}
void syncroImageClass::conbineImage(){
    syncImg.rgb = rgbImage;
    syncImg.depth = depthImage;
}
//publish
void syncroImageClass::publishRgbCamData(){//データ送信
    pub.publish(syncImg);
}
