#include<potbot_pcl/Clustering3D.h>

void Clustering3DClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("TOPIC/PCL2",TOPIC_PCL2);
    n.getParam("CLUSTERING_METHOD",CLUSTERING_METHOD);

}