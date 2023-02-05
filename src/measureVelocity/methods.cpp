#include<autonomous_mobile_robot_2022/measurementVelocity.h>

//subscribe
// void measurementVelocity::subscribeClusterData(){//クラスタデータ受信
// 	queue1.callOne(ros::WallDuration(1));
// }
void measurementVelocity::cluster_callback(const autonomous_mobile_robot_2022::ClassificationData::ConstPtr& msg)
{
    //データをコピー
    // curClstr.header = msg->header;
    // curClstr.width = msg->width;
    // curClstr.height = msg->height;
    // curClstr.res = msg->res;
    // curClstr.widthInt = msg->widthInt;
    // curClstr.heightInt = msg->heightInt;
    // curClstr.cp = msg->cp;
    // curClstr.size = msg->size;
    // curClstr.data = msg->data;
	curClstr = *msg;
    //manageに移動
    manage();
}
// void measurementVelocity::subscribeImageMatchingData(){//画像マッチングデータ受信
// 	queue2.callOne(ros::WallDuration(1));
// }
void measurementVelocity::matching_callback(const autonomous_mobile_robot_2022::ImageMatchingData::ConstPtr& msg)
{
    //データをコピー
    // matchData.header = msg->header;
    // matchData.width = msg->width;
    // matchData.height = msg->height;
    // matchData.res = msg->res;
    // matchData.widthInt = msg->widthInt;
    // matchData.heightInt = msg->heightInt;
    // matchData.cp = msg->cp;
    // matchData.index = msg->index;
    // matchData.data = msg->data;
	matchData = *msg;
    //manageに移動
    manage();
}
void measurementVelocity::manage(){
	// ROS_INFO("isCurClstr");
	if(isCurClstr() ){
		// ROS_INFO("creatClstrMap");
		creatClstrMap();
	}
	// ROS_INFO("isPrvClstr() && isMatchData()");
	if(isCurClstr() && isPrvClstr() && isMatchData()){
		// ROS_INFO("matchingClstr");
		matchingClstr();
		// ROS_INFO("measurementProcess");
		measurementProcess();
		//
		publishClassificationData();
		// ROS_INFO("visualizedVelocities");
		debug();
	}
	// ROS_INFO("renewClstrData");
	renewClstrData();
}
bool measurementVelocity::isCurClstr(){//curClstrのデータの有無
	if(curClstr.header.seq > 0 && (int)curClstr.data.size() > 0){
		return true;
	}
	return false;
}
bool measurementVelocity::isPrvClstr(){//prvClstrのデータの有無
	if(prvClstr.header.seq > 0 && (int)prvClstr.data.size() > 0){
		return true;
	}
	return false;
}
bool measurementVelocity::isMatchData(){//matchDataのデータの有無
	if(matchData.header.seq > 0 && (int)matchData.data.size()>0){
		return true;
	}
	return false;
}
void measurementVelocity::creatClstrMap(){//クラスタ番号を記述したマップを作成
	//マップセルに対応するクラスタ番号が記載
	// std::vector<int> curClstMap;
	curClstrMap.assign(curClstr.widthInt.data * curClstr.heightInt.data , -1);

	//マップの各セルにクラスタ番号を記載
	for(int i=0; i< curClstr.size.data; i++){
		for(int k=0; k<curClstr.data[i].index.size(); k++){
			curClstrMap[ curClstr.data[i].index[k].data ] = i;//セルにクラスタ番号を記述
		}
	}
}
void measurementVelocity::renewClstrData(){
	if(!isCurClstr()){
		return;
	}
	//データクリア（必要かどうかは不明）
	prvClstr.data.clear();
	prvClstrMap.clear();
	//データ更新
	prvClstrMap = curClstrMap;//クラスタマップ
	prvClstr = curClstr;//クラスタデータ
	//
	//データクリア（必要かどうかは不明）
	curClstr.data.clear();
	// curClstr.header.seq = 0;
	curClstrMap.clear();
    // matchData.index.clear();
    // matchData.data.clear();
}
void measurementVelocity::matchingClstr(){//（途中）
	//現在のクラスタ -> 1つ前の処理でのクラスタ に対するマッチングを取る
	//画像マッチングスコア
	//行：curClstrのクラスタ数, 列：prvClstrのクラスタ数, 値0で初期化
	std::vector<std::vector<int>> imageMathingScore(curClstr.size.data, std::vector<int>(prvClstr.size.data, 0));

	// ROS_INFO("matchData sousa");
	//matchData（画像マッチングデータ）を走査
	for(int k = 0; k < matchData.index.size(); k++){//widthInt.data * heightInt.data
		int prvPos = matchData.index[k].data;	
		if(prvPos < 0){
			continue;
		}
		int curPos = prvPos + matchData.data[prvPos].x.data
			+ matchData.data[prvPos].y.data * matchData.widthInt.data;
		if(curPos < 0){
			continue;
		}
		if(curClstrMap[curPos] < 0 || prvClstrMap[prvPos] < 0){
			continue;
		}
		// ROS_INFO("imageMathingScore[%d][%d]",curClstrMap[curPos], prvClstrMap[prvPos]);
		imageMathingScore[ curClstrMap[curPos] ][ prvClstrMap[prvPos] ] ++;//カウントアップ
	}
	// ROS_INFO("position");
	//位置マッチングスコア
	//重心位置の差
	std::vector<std::vector<float>> posMathingScore(curClstr.size.data, std::vector<float>(prvClstr.size.data, 0));
	for(int k=0; k < curClstr.size.data; k++){
		for(int i=0; i< prvClstr.size.data; i++){
			float disX = curClstr.data[k].gc.x - prvClstr.data[i].gc.x;
			float disY = curClstr.data[k].gc.y - prvClstr.data[i].gc.y;
			float disZ = curClstr.data[k].gc.z - prvClstr.data[i].gc.z;
			float dis = std::sqrt( disX*disX +disY*disY + disZ*disZ );
			posMathingScore[k][i] = dis;
		}
	}
	// ROS_INFO("size");
	//サイズマッチングスコア
	//サイズ（クラスタ内の点数）の差
	std::vector<std::vector<int>> sizeMathingScore(curClstr.size.data, std::vector<int>(prvClstr.size.data, 0));
	for(int k=0; k < curClstr.size.data; k++){
		for(int i=0; i< prvClstr.size.data; i++){
			int curSize = curClstr.data[k].dens.data;
			int prvSize = prvClstr.data[i].dens.data;
			sizeMathingScore[k][i] = std::abs(curSize - prvSize);
		}
	}
	// ROS_INFO("matching");
	// ROS_INFO_STREAM(curClstr.size.data <<","<<prvClstr.size.data);
	//マッチング評価
	// std::vector<int> matchResult(curClstr.size.data, -1);
	matchResult.assign(curClstr.size.data , -1);
	for(int k=0; k < curClstr.size.data; k++){
		//
		//例外除去
		//マップセルの占有マスが少ない（1マス以下）
		if(curClstr.size.data <= 1){
			continue;
		}
		double evMax = -1;//最大評価値
		int matchNum = -1;
		for(int i=0; i< prvClstr.size.data; i++){
			//例外除去
			//マップセルの占有マスが少ない（1マス以下）
			if(prvClstr.size.data <= 1){
				continue;
			}
			//重心位置の差が1m以上
			//--移動距離
			if(posMathingScore[k][i] >= 1.0){
				continue;
			}
			double ev;//評価値
			//評価式 : 要検討
			ev = weightImage * imageMathingScore[k][i] //画像マッチングスコア
				+ weightGravity * (1 / (1 + posMathingScore[k][i]) ) //重心位置マッチングスコア
				+ weightSize * (1 / (2 * (1 + sizeMathingScore[k][i])) );//サイズマッチングスコア
			//評価値が最大評価値よりも大きいとき
			if(evMax < ev){
				//ベストマッチング値を更新
				evMax = ev;
				matchNum = i;
			}
		}
		//評価が最もよかったやつをマッチング結果に
		matchResult[k] = matchNum;
	}
}
void measurementVelocity::measurementProcess(){//
	//速度データ付きのクラスタデータ
	// obstacle_detection::classificationVelocityData cvd;
    //データをコピー
    cvd.header = curClstr.header;
    cvd.width = curClstr.width;
    cvd.height = curClstr.height;
    cvd.res = curClstr.res;
    cvd.widthInt = curClstr.widthInt;
    cvd.heightInt = curClstr.heightInt;
    cvd.cp = curClstr.cp;
    cvd.size = curClstr.size;
    cvd.data = curClstr.data;
    // cvd.twist.resize(curClstr.size.data);
    cvd.twist.resize(curClstr.size.data);
    cvd.match.resize(curClstr.size.data);
    cvd.trackingNum.resize(curClstr.size.data);
	//経過時間の計算（速度算出用）
	double dt;
	ros::Duration rosDt = curClstr.header.stamp - prvClstr.header.stamp;
	dt = rosDt.toSec();
	cvd.dt = dt;
	//ROS_INFO(" curClstr.size.data, matchResult: %d, %d", curClstr.size.data, (int)matchResult.size());
	//ROS_INFO("cvd.twist: %d", (int)cvd.twist.size());
	std::vector<int> trackNumTemp;
	if(trackNum.size()){
		trackNumTemp.resize(trackNum.size());
		for(int k=0; k<trackNum.size();k++){
			trackNumTemp[k] = trackNum[k];
		}
	}
	trackNum = std::vector<int>(curClstr.size.data, 0);

	// std::cout<<"match result\n";
	for(int k=0; k < curClstr.size.data; k++){
		// std::cout<<k<<" -> "<< matchResult[k] <<": gp("<<curClstr.data[k].gc.x<<","<< curClstr.data[k].gc.y<<") <-- ";
		// std::cout<<k<<" -> "<< matchResult[k] <<": gp("<<curClstr.data[k].gc.x<<","<< curClstr.data[k].gc.y<<") ";
		cvd.match[k] = matchResult[k];
		if( matchResult[k] < 0){
			// std::cout<<"(NONE)\n";
			// cvd.twist[k].linear.x = 0;
			// cvd.twist[k].linear.y = 0;
			// cvd.twist[k].linear.z = 0;
			// cvd.twist[k].angular.x = 0;
			// cvd.twist[k].angular.y = 0;
			// cvd.twist[k].angular.z = 0;
			cvd.twist[k].twist.linear.x = 0;
			cvd.twist[k].twist.linear.y = 0;
			cvd.twist[k].twist.linear.z = 0;
			cvd.twist[k].twist.angular.x = 0;
			cvd.twist[k].twist.angular.y = 0;
			cvd.twist[k].twist.angular.z = atan2(-1.0,0.0);
		}
		else{
			//速度の計算
			//--移動距離
			float dx = curClstr.data[k].gc.x - prvClstr.data[ matchResult[k] ].gc.x;
			float dy = curClstr.data[k].gc.y - prvClstr.data[ matchResult[k] ].gc.y;
			float dz = curClstr.data[k].gc.z - prvClstr.data[ matchResult[k] ].gc.z;
			//--速度計算
			// std::cout<<" "<<dx<<"/"<< dt << " ";
			if(dt==0){
				// cvd.twist[k].linear.x = 0;
				// cvd.twist[k].linear.y = 0;
				// cvd.twist[k].linear.z = 0;
				cvd.twist[k].twist.linear.x = 0;
				cvd.twist[k].twist.linear.y = 0;
				cvd.twist[k].twist.linear.z = 0;
				// 変更　姿勢追加
				// cvd.twist[k].angular.x = 0;
				// cvd.twist[k].angular.y = 0;
				// cvd.twist[k].angular.z = 0;
				cvd.twist[k].twist.angular.x = 0;
				cvd.twist[k].twist.angular.y = 0;
				cvd.twist[k].twist.angular.z = atan2(-1.0,0.0);
			}
			else{
				// cvd.twist[k].linear.x = dx/dt;
				// cvd.twist[k].linear.y = dy/dt;
				// cvd.twist[k].linear.z = dz/dt;
				cvd.twist[k].twist.linear.x = dx/dt;
				cvd.twist[k].twist.linear.y = dy/dt;
				cvd.twist[k].twist.linear.z = dz/dt;
				// 変更　姿勢追加
				// cvd.twist[k].angular.x = 0;
				// cvd.twist[k].angular.y = 0;
				// cvd.twist[k].angular.z = atan2(dx,dy);
				cvd.twist[k].twist.angular.x = 0;
				cvd.twist[k].twist.angular.y = 0;
				cvd.twist[k].twist.angular.z = atan2(dy,dx);
			}
			//追跡回数インクリメント
			if(trackNumTemp.size()){
				trackNum[k] = trackNumTemp[matchResult[k]] + 1;
			}
			else{
				trackNum[k]++;
			}
			cvd.trackingNum[k] = trackNum[k];
			//コメント
			// std::cout<<"("<<prvClstr.data[matchResult[k]].gc.x<<","<< prvClstr.data[matchResult[k]].gc.y<<")\n";
			// std::cout<<"Vel("<<cvd.twist[k].linear.x<<","<< cvd.twist[k].linear.y<<")\n";
			// std::cout<<"Vel("<<cvd.twist[k].twist.linear.x<<","<< cvd.twist[k].twist.linear.y<<","<<cvd.twist[k].twist.angular.z<<")\n";//変更
			// ROS_INFO("trackNum[k]: %d", trackNum[k]);

		}
	}
}

void measurementVelocity::publishClassificationData(){//データ送信
    pub.publish(cvd);
}