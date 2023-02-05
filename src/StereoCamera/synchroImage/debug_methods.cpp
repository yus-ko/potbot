#include<potbot/synchroImage.h>

//デバッグタイプ選択
void syncroImageClass::debug(){
    switch(debugType){
        case 1: showSynchroPCL();break;
        default: ;
    }
}
//ポイントクラウド配信メソッド
void syncroImageClass::showSynchroPCL(){

    //表示用ポイントクラウド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewCloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    
    // sensor_msgs::Image rgbImage,depthImage;
    int width = syncImg.rgb.width;//画像横幅
    int height = syncImg.rgb.height;//画像縦幅    
    //conv bridgeimage
    cv_bridge::CvImagePtr bridgeImageRgb=cv_bridge::toCvCopy(syncImg.rgb,sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImagePtr bridgeImageDepth=cv_bridge::toCvCopy(syncImg.depth,sensor_msgs::image_encodings::TYPE_32FC1);
    //要素追加用の仮変数
	pcl::PointXYZRGB cloudTemp;
    //iter
    cv::Vec3b *it_rgb;
    float *it_dpt;
    int count = 0;
	viewCloud->points.resize(height*width);
    viewCloud->height = 1;
    viewCloud->width = count;
    // std::cout<<"for h "<<std::endl;
    for (int h=0; h<height; h++){
        it_rgb = bridgeImageRgb->image.ptr<cv::Vec3b>(h);
        it_dpt = bridgeImageDepth->image.ptr<float>(h);
        // std::cout<<"for w "<<std::endl;
        for(int w=0; w<width; w++){
            if(std::isnan(it_dpt[w]) || std::isinf(it_dpt[w])){
                // std::cout<<"if "<<std::endl;
            }
            else{
                // std::cout<<"else "<<std::endl;
                cloudTemp.b = it_rgb[w][0];
                cloudTemp.g = it_rgb[w][1];
                cloudTemp.r = it_rgb[w][2];
                cloudTemp.x=it_dpt[w];//y軸
                //y_temp=((float)hMax/2-h)*z_temp/f;//高さ算出 ■この計算式どこから？
                //x_temp=-( ((float)w-(float)wMax/2)*z_temp/f );
                cloudTemp.y= -( ((float)w-(float)width/2)*it_dpt[w]/f );//逆向きのx軸
                cloudTemp.z= ((float)height/2-h)*it_dpt[w]/f;//z軸
                //ポイントクラウドに追加
                viewCloud->points[count++] = cloudTemp;
                viewCloud->width = count;
            }
        }
    }
	viewCloud->points.resize(count);
	std::cout<<"viewCloud->points.size():"<<viewCloud->points.size()<<"\n";
    //データがないとき
	if(viewCloud->width <= 0)
	{
        ROS_INFO("No point cloud data!");
		return ;
	}
	sensor_msgs::PointCloud2 viewMsgs;
	pcl::toROSMsg (*viewCloud, viewMsgs);
	viewMsgs.header.frame_id="/zed_camera_center";
    //Publish
	pubDebPcl.publish(viewMsgs);

}
