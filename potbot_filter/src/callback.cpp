#include<potbot_filter/Filter.h>

void FilterClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
    //print_Pose(odom_.pose.pose);
}

void matrixToDoubleArray(const Eigen::MatrixXd& matrix, std_msgs::Float64MultiArray& array) {
    int rows = matrix.rows();
    int cols = matrix.cols();

    int index = 0;
    for (int j = 0; j < cols; j++) {
        for (int i = 0; i < rows; i++) {
            array.data[index] = matrix(i, j);
            index++;
        }
    }
}

#define __NX__ 5
#define __NY__ 2


Eigen::VectorXd f(Eigen::VectorXd x_old, double dt) {
	Eigen::VectorXd x_new(__NX__);

	x_new(0) = x_old(0) + x_old(3)*cos(x_old(2))*dt;
	x_new(1) = x_old(1) + x_old(3)*sin(x_old(2))*dt;
	x_new(2) = x_old(2) + x_old(4)*dt;
	x_new(3) = x_old(3);
	x_new(4) = x_old(4);

	// x_new(0) = x_old(0) + x_old(2)*dt;
	// x_new(1) = x_old(1) + x_old(3)*dt;
	// x_new(2) = x_old(2);
	// x_new(3) = x_old(3);

	return x_new;
}

Eigen::VectorXd h(Eigen::VectorXd x, double dt) {
	Eigen::VectorXd y(__NY__);
	y(0) = x(0);
	y(1) = x(1);
	return y;
}

int id2index(int id, std::vector<int> idvec)
{
    int idx = -1;
    for(int i = 0; i < idvec.size(); i++)
    {
        if (idvec[i] == id)
        {
            idx = i;
            break;
        }
    }
    return idx;
}

void FilterClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles_ = msg;
    static std::vector<int> ukf_id;
    //ROS_INFO("__obstacle_callback");

    if (obstacles_.markers.size() > 0)
    {
        double t_now = obstacles_.markers[0].header.stamp.toSec();  //markers配列のサイズが0のときの例外処理を追加する
        // double t_now = ros::Time::now().toSec();
        static double t_pre = t_now;
        double dt = t_now-t_pre;

        static tf2_ros::TransformListener tfListener(tf_buffer_);

        potbot_msgs::StateArray state_array_msg;
        for(int i = 0; i < obstacles_.markers.size(); i++)
        {
            
            if (obstacles_.markers[i].ns == "segments_display")
            {
                geometry_msgs::TransformStamped transform;
                geometry_msgs::PointStamped target_point;
                try 
                {
                    // 2つの座標系間の変換を取得
                    transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, obstacles_.markers[i].header.frame_id, ros::Time());
                    geometry_msgs::PointStamped source_point;
                    // source_point.header = obstacles_.markers[i].header;
                    // source_point.header.frame_id = "lidar";
                    // source_point.header.stamp = ros::Time();
                    source_point.point = obstacles_.markers[i].pose.position;

                    tf2::doTransform(source_point, target_point, transform);

                    // ROS_INFO("Transformed Point: (%f, %f, %f) in target_frame",
                    //         target_point.point.x, target_point.point.y, target_point.point.z);
                }
                catch (tf2::TransformException &ex) 
                {
                    ROS_ERROR("TF Ereor in FilterClass::__obstacle_callback: %s", ex.what());
                    break;
                }

                double x = target_point.point.x;
                double y = target_point.point.y;
                int id = obstacles_.markers[i].id;

                auto iter = std::find(ukf_id.begin(), ukf_id.end(), id);
                if (iter != ukf_id.end())
                {
                    int index_ukf = std::distance(ukf_id.begin(), iter);
                    int ny = 2;
                    Eigen::VectorXd observed_data(ny);
                    observed_data<< x, y;
                    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = states_ukf_[index_ukf].update(observed_data,dt);
                    Eigen::MatrixXd xhat = std::get<0>(ans);
                    Eigen::MatrixXd P = std::get<1>(ans);
                    Eigen::MatrixXd K = std::get<2>(ans);

                    potbot_msgs::State state_msg;
                    state_msg.header = obstacles_.markers[i].header;
                    state_msg.header.frame_id = FRAME_ID_GLOBAL;
                    state_msg.id = id;

                    state_msg.z.data.resize(ny);
                    state_msg.xhat.data.resize(xhat.rows());
                    state_msg.P.data.resize(P.rows()*P.cols());
                    state_msg.K.data.resize(K.rows()*K.cols());

                    matrixToDoubleArray(observed_data, state_msg.z);
                    matrixToDoubleArray(xhat, state_msg.xhat);
                    matrixToDoubleArray(P, state_msg.P);
                    matrixToDoubleArray(K, state_msg.K);
                    
                    //std::cout<<xhat.transpose()<<std::endl;
                    state_array_msg.data.push_back(state_msg);
                }
                else
                {
                    
                    Eigen::MatrixXd Q(__NY__,__NY__), R(__NX__,__NX__), P(__NX__,__NX__);
                    Q.setZero();R.setZero();P.setZero();
                    // for (int i = 0; i < __NY__; i++) Q(i,i) = SIGMA_Q;
                    // for (int i = 0; i < __NX__; i++) R(i,i) = SIGMA_R;
                    // for (int i = 0; i < __NX__; i++) P(i,i) = SIGMA_P;

                    Q<< SIGMA_Q, 0,
                        0, SIGMA_Q;
                    
                    R<< 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, SIGMA_R, 0,
                        0, 0, 0, 0, SIGMA_R;

                    P<< SIGMA_P, 0, 0, 0, 0,
                        0, SIGMA_P, 0, 0, 0,
                        0, 0, SIGMA_P, 0, 0,
                        0, 0, 0, SIGMA_P, 0,
                        0, 0, 0, 0, SIGMA_P;

                    Eigen::VectorXd xhat(__NX__);
                    // xhat.setZero();
                    xhat<< x,y,0,0,0;
                    UKF estimate(f,h,R,Q,P,xhat);

                    states_ukf_.push_back(estimate);
                    ukf_id.push_back(id);
                }

                // ROS_INFO("%f, %f",x,y);

            }
        }
        t_pre = t_now;
        
        if (state_array_msg.data.size() > 0) pub_state_.publish(state_array_msg);
    }

    
}

void FilterClass::__scan_callback(const sensor_msgs::LaserScan& msg)
{
    std::vector<std::vector<double>> color = {
                                                {1,0,0},
                                                {0,1,0},
                                                {0,0,1},
                                                {1,1,0},
                                                {1,0,1},
                                                {0,1,1},
                                                {0,0,0}
                                            };
    //ROS_INFO("scan callback");

    scan_ = msg;

    scan_.header.frame_id = FRAME_ID_LIDAR;
    pub_scan0_.publish(scan_);
    __MedianFilter(scan_);
    pub_scan1_.publish(scan_);
    std::vector<SEGMENT> segments;
    __Segmentation(scan_, segments);
    __SplitSegments(segments);
    __AssociateSegments(segments);

    // for(int i = 0; i < segments.size(); i++) std::cout<<segments[i].id<<", ";
    // std::cout<<std::endl;

    visualization_msgs::MarkerArray seg;
    for (int i = 0; i < segments.size(); i++)
    {

        visualization_msgs::Marker segment;
        segment.header = scan_.header;
        segment.header.frame_id = FRAME_ID_LIDAR;

        segment.ns = "segments_display";
        segment.id = segments[i].id;
        segment.lifetime = ros::Duration(1);

        segment.type = segments[i].type;
        segment.action = visualization_msgs::Marker::MODIFY;

        
        segment.pose.position.x = segments[i].x;
        segment.pose.position.y = segments[i].y;
        segment.pose.position.z = 0;

        segment.pose.orientation.x = 0;
        segment.pose.orientation.y = 0;
        segment.pose.orientation.z = 0;
        segment.pose.orientation.w = 1;

        if (segment.type == visualization_msgs::Marker::SPHERE)
        {
            segment.scale.x = segments[i].radius*2;
            segment.scale.y = segments[i].radius*2;
        }
        else if (segment.type == visualization_msgs::Marker::CUBE)
        {
            segment.scale.x = segments[i].width;
            segment.scale.y = segments[i].height;
        }

        segment.scale.z = 0.001;

        segment.color.a = 0.3;

        segment.color.r = color[i%color.size()][0];
        segment.color.g = color[i%color.size()][1];
        segment.color.b = color[i%color.size()][2];
        
        seg.markers.push_back(segment);

        for (int j = 0; j < segments[i].points.size(); j++)
        {
            visualization_msgs::Marker point;
            point.header = segment.header;

            point.ns = "points_display";
            point.id = segments[i].id;
            point.lifetime = ros::Duration(1);

            point.type = segment.type;
            point.action = visualization_msgs::Marker::ADD;

            
            point.pose.position.x = segments[i].points[j].x;
            point.pose.position.y = segments[i].points[j].y;
            point.pose.position.z = 0;

            point.pose.orientation = segment.pose.orientation;

            point.scale.x = 0.02;
            point.scale.y = 0.02;
            point.scale.z = 0.001;

            point.color.a = 1;

            point.color.r = segment.color.r;
            point.color.g = segment.color.g;
            point.color.b = segment.color.b;
            
            seg.markers.push_back(point);
        }
        
    }
    pub_segment_.publish(seg);
}

void FilterClass::__param_callback(const potbot_filter::FilterConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    Tn_ = param.threshold_point_num;
    square_width_ = param.squre_width;
}