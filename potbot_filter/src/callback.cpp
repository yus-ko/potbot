#include<potbot_filter/Filter.h>

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

void FilterClass::__obstacle_callback(const potbot_msgs::ObstacleArray& msg)
{
    potbot_msgs::ObstacleArray obstacle_array = msg;
    static std::vector<int> ukf_id;
    //ROS_INFO("__obstacle_callback");
    if (obstacle_array.data.empty()) return;

    double t_now = obstacle_array.header.stamp.toSec();  //markers配列のサイズが0のときの例外処理を追加する
    // double t_now = ros::Time::now().toSec();
    static double t_pre = t_now;
    double dt = t_now-t_pre;

    static tf2_ros::TransformListener tfListener(tf_buffer_);

    potbot_msgs::StateArray state_array_msg;
    // for(int i = 0; i < obstacle_array.data.size(); i++)
    for(auto& obstacle : obstacle_array.data)
    {
        
        geometry_msgs::TransformStamped transform;
        geometry_msgs::PointStamped target_point;
        try 
        {
            // 2つの座標系間の変換を取得
            transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, obstacle.header.frame_id, ros::Time());
            geometry_msgs::PointStamped source_point;
            source_point.point = obstacle.pose.position;

            tf2::doTransform(source_point, target_point, transform);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_ERROR("TF Ereor in FilterClass::__obstacle_callback: %s", ex.what());
            break;
        }

        double x = target_point.point.x;
        double y = target_point.point.y;
        int id = obstacle.id;

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
            state_msg.header = obstacle.header;
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

            obstacle.twist.linear.x = state_msg.xhat.data[3];
            obstacle.twist.angular.z = state_msg.xhat.data[4];
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
            potbot_lib::UnscentedKalmanFilter estimate(f,h,R,Q,P,xhat);

            states_ukf_.push_back(estimate);
            ukf_id.push_back(id);
        }

        // ROS_INFO("%f, %f",x,y);

    }
    t_pre = t_now;

    pub_obstacles_scan_.publish(obstacle_array);
    
    if (!state_array_msg.data.empty())
    {
        visualization_msgs::MarkerArray state_markers;

        for (const auto& state_msg : state_array_msg.data)
        {
            if (abs(state_msg.xhat.data[3]) > 2.0 || abs(state_msg.xhat.data[3]) < 0.1) continue;

            visualization_msgs::Marker state_marker;

            state_marker.header             = state_msg.header;

            state_marker.ns                 = "segments/centor";
            state_marker.id                 = state_msg.id;
            state_marker.lifetime           = ros::Duration(1);

            state_marker.type               = visualization_msgs::Marker::ARROW;
            state_marker.action             = visualization_msgs::Marker::MODIFY;

            state_marker.pose.position.x    = state_msg.xhat.data[0];
            state_marker.pose.position.y    = state_msg.xhat.data[1];
            state_marker.pose.position.z    = 0;

            state_marker.pose.orientation   = potbot_lib::utility::get_Quat(0,0,state_msg.xhat.data[2]);

            state_marker.scale.x            = 0.05;
            state_marker.scale.y            = 0.1;
            state_marker.scale.z            = 0.1;
            
            state_marker.color              = potbot_lib::color::get_msg(state_msg.id);
            state_marker.color.a            = 0.5;

            geometry_msgs::Point p0, p1;
            p0.x                            = 0;
            p0.y                            = 0;
            p0.z                            = 0;
            p1.x                            = state_msg.xhat.data[3];
            p1.y                            = 0;
            p1.z                            = 0;
            state_marker.points.push_back(p0);
            state_marker.colors.push_back(state_marker.color);
            state_marker.points.push_back(p1);
            state_marker.colors.push_back(state_marker.color);
            
            state_markers.markers.push_back(state_marker);

        }

        pub_state_markers_.publish(state_markers);
        pub_state_.publish(state_array_msg);
    }

    // if (!states_ukf_.empty())
    // {
    //     potbot_msgs::ObstacleArray obstacle_array;
    //     obstacle_array.header.frame_id = FRAME_ID_GLOBAL;
    //     obstacle_array.header.stamp = ros::Time(0);
    //     for (auto& state : states_ukf_)
    //     {
    //         potbot_msgs::Obstacle obstacle_msg;
    //         nav_msgs::Odometry obstacle_odom;
    //         state.get_odom_state(obstacle_odom);
            
    //         obstacle_msg.header = obstacle_array.header; 
    //         obstacle_msg.id = 1;
    //         obstacle_msg.pose = obstacle_odom.pose.pose;
    //         obstacle_msg.twist = obstacle_odom.twist.twist;

    //         obstacle_array.data.push_back(obstacle_msg);

    //     }
    //     pub_obstacles_scan_.publish(obstacle_array);
    // }

}

void FilterClass::__scan_callback(const sensor_msgs::LaserScan& msg)
{
    //ROS_INFO("scan callback: filter");

    scan_ = msg;

    pub_scan0_.publish(scan_);
    //__MedianFilter(scan_);
    pub_scan1_.publish(scan_);
    std::vector<SEGMENT> segments;
    __Segmentation(scan_, segments);
    __SplitSegments(segments);
    __AssociateSegments(segments);

    // for(int i = 0; i < segments.size(); i++) std::cout<<segments[i].id<<", ";
    // std::cout<<std::endl;

    visualization_msgs::MarkerArray seg;
    potbot_msgs::ObstacleArray obstacle_array_msg;
    obstacle_array_msg.header = scan_.header;
    std::vector<int> ids;
    ids.reserve(segments.size()); // 適切なサイズを確保
    // transformを使用してidsベクトルにidだけを取り出す
    std::transform(segments.begin(), segments.end(), std::back_inserter(ids),
                   [](const SEGMENT& segment) { return segment.id; });

    int point_id = *(std::max_element(ids.begin(), ids.end())) + 1;
    // ROS_INFO("init point_id %d",point_id);
                   
    for (int i = 0; i < segments.size(); i++)
    {

        visualization_msgs::Marker segment;
        segment.header = scan_.header;

        segment.ns = "segments/centor";
        segment.id = segments[i].id;
        segment.lifetime = ros::Duration(1);

        segment.type = segments[i].type;
        segment.action = visualization_msgs::Marker::MODIFY;

        segment.pose.position.x = segments[i].x;
        segment.pose.position.y = segments[i].y;
        segment.pose.position.z = 0;

        segment.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);

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

        segment.color = potbot_lib::color::get_msg(segment.id);
        segment.color.a = 0.3;
        
        seg.markers.push_back(segment);

        visualization_msgs::Marker point = segment;
        point.ns = "segments/points";
        point.id = point_id++;
        point.type = visualization_msgs::Marker::SPHERE_LIST;
        point.action = visualization_msgs::Marker::ADD;
        point.pose.position.x = 0;
        point.pose.position.y = 0;
        point.pose.position.z = 0;
        point.scale.x = 0.01;
        point.scale.y = 0.01;
        point.scale.z = 0.01;

        for (int j = 0; j < segments[i].points.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = segments[i].points[j].x;
            p.y = segments[i].points[j].y;
            p.z = 0;

            point.points.push_back(p);
            point.colors.push_back(point.color);
            
        }
        seg.markers.push_back(point);

        potbot_msgs::Obstacle obstacle_msg;
        obstacle_msg.header = segment.header;
        obstacle_msg.id = segment.id;
        obstacle_msg.pose = segment.pose;
        obstacle_msg.scale = segment.scale;
        obstacle_msg.points = point.points;
        obstacle_array_msg.data.push_back(obstacle_msg);
        
    }
    pub_segment_.publish(seg);
    pub_obstacles_scan_test_.publish(obstacle_array_msg);
}

void FilterClass::__param_callback(const potbot_filter::FilterConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    Tn_                     = param.threshold_point_num;
    square_width_           = param.squre_width;
}