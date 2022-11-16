#include<autonomous_mobile_robot_2022/convCamData.h>

//subscribe
bool convCamDataClass::subscribeSensorData(){//データ受信
    ros::CallbackQueue::CallOneResult res;
	res = queue.callOne(ros::WallDuration(1));
    // ROS_INFO_STREAM("subscribe result : " << res);
    if(res == ros::CallbackQueue::CallOneResult::Called){return true;}
    return false;
}

void convCamDataClass::sensor_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try{
        // bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
        bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);//CV_16UC1
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        std::cout<<"sensor_depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'MONO16'.",
        msg->encoding.c_str());
        return ;
    }
    //追加
    manage();
}

//manage method
void convCamDataClass::manage(){
    create2dMap();
    publishConvCamData();
    publishMaskImage();
}

void convCamDataClass::create2dMap(){
    //ステレオカメラパラメータ
    int hMax = bridgeImage->image.rows;//行480
    int wMax = bridgeImage->image.cols;//列752
    int ch = bridgeImage->image.channels();//チャンネル数
    // cv::Mat img8(hMax, wMax, CV_8UC1);
    // bridgeImage->image.convertTo(img8, CV_8UC1, 1/255.0);
    //仮変数
    float x_temp;
    float y_temp;
    float z_temp;
    //マップデータ
    smd.header = bridgeImage->header;
    smd.width.data = mapW;
    smd.height.data = mapH;
    smd.res.data = mapR;
    smd.widthInt.data = (int)(mapW/mapR);
    smd.heightInt.data = (int)(mapH/mapR);
    smd.cp.x=0;
    smd.cp.y=0;
    smd.cp.z=0;
    smd.index.resize(smd.heightInt.data*smd.widthInt.data);
    smd.size.resize(smd.heightInt.data*smd.widthInt.data);
    smd.pt.resize(smd.heightInt.data*smd.widthInt.data);
    // smd.index.resize(hMax*wMax);
    // smd.pt.resize(hMax*wMax);
    //マスクデータ
    mid.header = bridgeImage->header;
    //--画像データ
    mid.height.data = hMax;
    mid.width.data = wMax;
    //--マップデータ
    mid.mapWidth.data = mapW;
    mid.mapHeight.data = mapH;
    mid.mapRes.data = mapR;
    mid.mapWidthInt.data = (int)(mapW/mapR);
    mid.mapHeightInt.data = (int)(mapH/mapR);
    mid.pt.resize(wMax*hMax);
    //--インデックスデータ(maskImageData)
    // 画像ピクセル位置 -> マップデータへの対応付け
    // 画像ピクセル(h,w) -> マップデータ
    // index ==-1 のとき、対応するデータがないことを示す
    // 参照方法
    // pt[ index[width*h+w] ]
    //
    mid.index.resize(wMax*hMax);
    //--インデックスデータ(sensorMapData)
    //インデックス初期化
    for(int h=0;h<smd.heightInt.data;h++){
        for(int w=0;w<smd.widthInt.data;w++){
            smd.index[h*smd.widthInt.data+w].data=-1;
            smd.size[h*smd.widthInt.data+w].data=0;
            smd.pt[h*smd.widthInt.data+w].x = 0;
            smd.pt[h*smd.widthInt.data+w].y = 0;
            smd.pt[h*smd.widthInt.data+w].z = 0;
        }
    }
    int countMid=0;
    int countSmd=0;
    //2次元ローカルマップの作成
    for(int h=0;h<hMax;h++){//画像すべてを走査
        //画素高速アクセス用パラメータ(先頭アドレス)
        // float *p = bridgeImage->image.ptr<float>(h);//ポインタ取得
        for(int w=0;w<wMax;w++){
            // z_temp = p[w*ch];
            // z_temp=bridgeImage->image.at<float>(h,w);
            z_temp=(bridgeImage->image.at<unsigned short int>(h,w))/1000.0;
            //if(z_temp>0) {ROS_INFO_STREAM("z_temp_int:"<<z_temp);}
            if(z_temp>0&&!std::isinf(z_temp)){
                y_temp=((float)hMax/2-h)*z_temp/f+camHeight/1000;//高さ算出
                // ROS_INFO_STREAM("highth:"<<(float)hMax-h);
                x_temp=-((float)w-(float)wMax/2)*z_temp/f;
                // ROS_INFO_STREAM("width:"<<(float)w-(float)wMax/2);
                // if(y_temp>0) {ROS_INFO_STREAM("y_temp:"<<y_temp);}
                // ROS_INFO_STREAM("x_temp:"<<x_temp);
                //高さが0以下の範囲を検出
                if( y_temp <=0 ){
                    //マスクデータ格納
                    mid.index[h*wMax+w].data = -1;
                    // ROS_INFO_STREAM("y<0:"<<mid.index[h*wMax+w].data);
                }
                else{
                    //データ格納
                    int xi,zi;//仮変数
                    if(convertToGrid(x_temp,z_temp,xi,zi)){//データ変換
                        //インデックスデータ
                        int index = smd.index[zi*smd.widthInt.data+xi].data;
                        //データがすでに格納されているか
                        if(index < 0){
                            //データがまだ無い場合
                            smd.index[zi*smd.widthInt.data+xi].data = countSmd;
                            index = countSmd++;//データ数をインクリメント, indexをsmd.index配列の末尾に設定 
                        }
                        //データサイズのインクリメント
                        smd.size[index].data+=1;
                        //マップデータ格納
                        smd.pt[index].x += x_temp;//横方向
                        smd.pt[index].y += z_temp;//奥行
                        smd.pt[index].z += y_temp;//高さ
                        //
                        //マスクデータ格納
                        mid.index[h*wMax+w].data = countMid;
                        mid.pt[countMid].x=x_temp;//横方向
                        mid.pt[countMid].y=z_temp;//奥行
                        mid.pt[countMid].z=y_temp;//高さ
                        //Midのデータ数をインクリメント
                        countMid++;
                    }
                    else{
                        //マスクデータ格納
                        mid.index[h*wMax+w].data = -1;
                        // ROS_INFO_STREAM("maskdata:"<<mid.index[h*wMax+w].data);
                    }
                }
            }
            else{
                //マスクデータ格納
                mid.index[h*wMax+w].data = -1;
                // ROS_INFO_STREAM("nodata:"<<mid.index[h*wMax+w].data);
            }
        }
    }
    //サイズ調整
    // smd.index.resize(k);
    smd.pt.resize(countSmd);
    smd.size.resize(countSmd);
    mid.pt.resize(countMid);
    //smdデータptの平均値を算出
    for(int k = 0; k < smd.size.size(); k++){
        if(smd.size[k].data > 0){
            //データサイズで割る
            smd.pt[k].x /= smd.size[k].data;//横方向
            smd.pt[k].y /= smd.size[k].data;//奥行
            smd.pt[k].z /= smd.size[k].data;//高さ
        }
    }
}

bool convCamDataClass::convertToGrid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標(ふつうは　センサ位置＝マップ中心座標　のため　cx=cy=0)
	float cx=0;
	float cy=0;
    //マップ原点座標を画像原点座標(左上)に移動
	float map_x = mapW/2 + (x - cx);
	float map_y = mapH/2 + ( -(y - cy) );
    //マップ外のデータの際はreturn false
	if(map_x<0 || map_x>mapW)
		return false;
	if(map_y<0 || map_y>mapH)
		return false;
    //ピクセルデータに変換
	xg = (int)(map_x/mapR);
	yg = (int)(map_y/mapR);
    //変換成功
	return true;
}

void convCamDataClass::publishConvCamData(){//データ送信
    pubConv.publish(smd);
}

void convCamDataClass::publishMaskImage(){//データ送信
    pubMask.publish(mid);
}

void convCamDataClass::clearMessages(){
    smd.index.clear();
    smd.pt.clear();
    smd.size.clear();
    mid.index.clear();
    mid.pt.clear();
}
