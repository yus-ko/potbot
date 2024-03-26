#include<potbot_filter/2dscan_clustering.h>


int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_fi");

	ros::NodeHandle nh("~");
	double default_param = 20.0;
	// nh.param<double>("SIGMA/P", default_param, 10.0);
	// nh.getParam("SIGMA/A", default_param);
	nh.param<double>("SIGMA/B", default_param, 30.0);
	std::cout<<default_param<<std::endl;

    scan2dClass s2d;
	s2d.mainloop();
	
	return 0;
}

scan2dClass::scan2dClass()
{
	__get_param();
	
	sub_scan_						= nhSub_.subscribe(TOPIC_SCAN,							1,&scan2dClass::__scan_callback,this);
	
	pub_scan_filter_				= nhPub_.advertise<sensor_msgs::LaserScan>(				TOPIC_SCAN+"/filter", 1);
	pub_segment_					= nhPub_.advertise<visualization_msgs::MarkerArray>(	"segment", 1);
	pub_obstacles_scan_clustering_	= nhPub_.advertise<potbot_msgs::ObstacleArray>(			"obstacle/scan/clustering", 1);

	f_ = boost::bind(&scan2dClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

	static tf2_ros::TransformListener tfListener(tf_buffer_);
	while (true)
	{
		try
		{
			tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, ros::Time(0), ros::Duration(60));
			break;
		}
		catch (tf2::TransformException &ex) 
		{
			ROS_WARN("tf unavailable: %s", ex.what());
		}
	}
}

void scan2dClass::__get_param()
{
	ros::NodeHandle n("~");
	n.getParam("FRAME_ID/GLOBAL",       FRAME_ID_GLOBAL);
    n.getParam("FRAME_ID/ROBOT_BASE",   FRAME_ID_ROBOT_BASE);
    n.getParam("TOPIC/SCAN",            TOPIC_SCAN);
}

void scan2dClass::mainloop()
{
	ros::spin();
}

void scan2dClass::__param_callback(const potbot_msgs::ClusteringConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    Tn_                     = param.threshold_point_num;
    square_width_           = param.squre_width;
}

void scan2dClass::__scan_callback(const sensor_msgs::LaserScan::ConstPtr msg)
{
	//ROS_INFO("scan callback: filter");

    scan_ = *msg;

    // __MedianFilter(scan_);
    pub_scan_filter_.publish(scan_);
    std::vector<SEGMENT> segments;

    potbot_lib::ScanClustering scanclus;
    scanclus.set_clusters(scan_);   //センサーデータの登録
    scanclus.euclidean_clustering();    //ユークリッド距離に基づいたクラスタリングを実行

    potbot_msgs::ObstacleArray clusters_obstaclearray_scan;
    scanclus.to_obstaclearray(clusters_obstaclearray_scan);    //クラスタリング結果をpotbot_msgs::ObstacleArray型に変換して取得
    clusters_obstaclearray_scan.header = scan_.header;
    for (auto& obs : clusters_obstaclearray_scan.data) obs.header = clusters_obstaclearray_scan.header;

    //1時刻前のクラスタからの追跡
    static potbot_msgs::ObstacleArray clusters_obstaclearray_pre;
    potbot_lib::utility::associate_obstacle(clusters_obstaclearray_scan, clusters_obstaclearray_pre, tf_buffer_);

    scanclus.set_clusters(clusters_obstaclearray_scan);

    //クラスタをワールド座標系に変換して1時刻先の追跡用データにする
    potbot_lib::utility::get_tf(tf_buffer_, clusters_obstaclearray_scan, FRAME_ID_GLOBAL, clusters_obstaclearray_pre);

    //クラスタをロボット座標系に変換
    potbot_msgs::ObstacleArray clusters_obstaclearray_robot;
    potbot_lib::utility::get_tf(tf_buffer_, clusters_obstaclearray_scan, FRAME_ID_ROBOT_BASE, clusters_obstaclearray_robot);

    visualization_msgs::MarkerArray clusters_markerarray;
    scanclus.to_markerarray(clusters_markerarray);  //クラスタリング結果をvisualization_msgs::MarkerArray型に変換して取得
    for (auto& clus : clusters_markerarray.markers) clus.header = scan_.header;

    pub_segment_.publish(clusters_markerarray);
    pub_obstacles_scan_clustering_.publish(clusters_obstaclearray_robot);
    
    return;


    __Segmentation(scan_, segments);
    __SplitSegments(segments);
    __AssociateSegments(segments);

    // for(int i = 0; i < segments.size(); i++) std::cout<<segments[i].id<<", ";
    // std::cout<<std::endl;

    visualization_msgs::MarkerArray seg;
    potbot_msgs::ObstacleArray obstacle_array_msg;
    obstacle_array_msg.header = scan_.header;
    obstacle_array_msg.header.frame_id = FRAME_ID_ROBOT_BASE;
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
        point.pose.position = potbot_lib::utility::get_Point(0,0,0);
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



        geometry_msgs::PoseStamped segment_scan;
        potbot_msgs::Obstacle obstacle_msg;
        segment_scan.header = segment.header;
        segment_scan.pose = segment.pose;

        obstacle_msg.header = obstacle_array_msg.header;
        obstacle_msg.id = segment.id;
		obstacle_msg.is_moving = segments[i].is_moving;
        obstacle_msg.pose = potbot_lib::utility::get_tf(tf_buffer_, segment_scan, obstacle_msg.header.frame_id).pose;
        obstacle_msg.scale = segment.scale;
        std::vector<geometry_msgs::Point> points_scan;
        for (const auto& point : point.points)
        {
            geometry_msgs::PoseStamped point_scan;
            point_scan.header = segment_scan.header;
            point_scan.pose.position = point;
            point_scan.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
            geometry_msgs::Pose point_robot = potbot_lib::utility::get_tf(tf_buffer_, point_scan, obstacle_msg.header.frame_id).pose;
            points_scan.push_back(point_robot.position);
        }

        obstacle_msg.points = points_scan;
        obstacle_array_msg.data.push_back(obstacle_msg);
        
    }
    pub_segment_.publish(seg);
    pub_obstacles_scan_clustering_.publish(obstacle_array_msg);
}

double scan2dClass::__Median(std::vector<double> v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    if (v.size() % 2 == 1) 
    {
        return v[n];
    } else 
    {
        std::nth_element(v.begin(), v.begin() + n - 1, v.end());
        return (v[n - 1] + v[n]) / 2.0;
    }
}

void scan2dClass::__MedianFilter(sensor_msgs::LaserScan &scan)
{
    int window_num = 3;
    scans_.push_back(scan);
    int scans_size = scans_.size();
    if (scans_size >= window_num)
    {
        int scan_size = scan.ranges.size();
        for (int i = 1; i < scan_size-1; i++)
        {
            std::vector<double> scan_data;
            for (int t = scans_size - window_num; t < scans_size; t++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    double data = scans_[t].ranges[i+j];
                    // if (std::isinf(data) || std::isnan(data)) data = std::numeric_limits<double>::infinity();
                    if (data < scans_[t].range_min) data = scans_[t].range_min;
                    else if (data > scans_[t].range_max) data = scans_[t].range_max;
                    scan_data.push_back(data);
                }
            }
            scan.ranges[i] = __Median(scan_data);
        }
        scans_.erase(scans_.begin());
    }
}

void scan2dClass::__Segmentation(sensor_msgs::LaserScan &scan, std::vector<SEGMENT> &segments)
{
    int size = scan.ranges.size();
    bool start = false;
    SEGMENT seg;
    seg.type = visualization_msgs::Marker::SPHERE;
    for (int i = 0; i < size; i++)
    {
        
        //if (!std::isinf(scan.ranges[i]) && !std::isnan(scan.ranges[i]))
        if (scan_.range_min <= scan_.ranges[i] && scan_.ranges[i] <= scan_.range_max)
        {
            if(!start)
            {
                start = true;
                seg.points.resize(0);
            }
            POINT p;
            p.index = i;
            p.theta = p.index * scan_.angle_increment + scan_.angle_min;
            p.r = scan_.ranges[i] + scan_.range_min;
            p.x = p.r * cos(p.theta);
            p.y = p.r * sin(p.theta);

            if (seg.points.size() > 0)
            {    
                POINT &p_pre = seg.points.back();

                double distance = sqrt(pow(p.x - p_pre.x,2) + pow(p.y - p_pre.y,2));
                if (distance <= 0.3)
                {
                    seg.points.push_back(p);
                }
                else
                {
                    start = false;
                    segments.push_back(seg);
                    i--;
                    continue;
                }
            }
            else
            {
                seg.points.push_back(p);
            }

            if (i == size - 1 && start)
            {
                segments.push_back(seg);
            }
        }
        else if(start)
        {
            start = false;
            segments.push_back(seg);
        }
    }
}

double scan2dClass::__distanceToLineSegment(POINT o, POINT p, POINT q)
{
    // double ABx = q.x - p.x;
    // double ABy = q.y - p.y;
    // double ABlength = sqrt(pow(ABx, 2) + pow(ABy, 2));
    // double ABx_norm = ABx / ABlength;
    // double ABy_norm = ABy / ABlength;
    // double APx = o.x - p.x;
    // double APy = o.y - p.y;
    // double APdistance = sqrt(pow(APx, 2) + pow(APy, 2));
    // double dotProduct = APx * ABx_norm + APy * ABy_norm;
    // double xd = p.x + dotProduct * ABx_norm;
    // double yd = p.y + dotProduct * ABy_norm;
    // double distance = sqrt(pow(o.x - xd, 2) + pow(o.y - yd, 2));

    // double a = (q.y - p.y)/(q.x - p.x);
    // double b = 1;
    // double c = -p.y;
    // double distance = abs(o.x*a + b*o.y + c) / sqrt(a*a+b*b);

    double theta = atan2(o.y-p.y, o.x-p.x) - atan2(q.y-p.y, q.x-p.x);
    // double theta = acos((q.x*p.x + q.y*p.y) / (sqrt(q.x*q.x + q.y*q.y) * sqrt(p.x*p.x + p.y*p.y)));
    double l = sqrt(pow(o.x - p.x, 2) + pow(o.y - p.y, 2));
    double distance = l*sin(theta);

    return distance;
}

void scan2dClass::__SplitSegments(std::vector<SEGMENT> &segments)
{
    std::vector<SEGMENT> segments_original = segments;   //Vc
    segments.resize(0); //Vresult

    
    double square_width = square_width_;

    while(segments_original.size() != 0)
    {
        int Nc0 = segments_original[0].points.size();
        if (Nc0 > 2)
        {
            POINT p = segments_original[0].points.front();
            POINT q = segments_original[0].points.back();
            std::vector<double> distance;
            for (int i = 1; i < Nc0-1; i++)
            {
                double d = __distanceToLineSegment(segments_original[0].points[i], p, q);
                distance.push_back(d);
            }
            std::vector<double>::iterator max_itr = std::max_element(distance.begin(), distance.end());
            double Dm = *max_itr;
            double S = sqrt(pow(q.x - p.x,2) + pow(q.y - p.y,2));

            segments_original[0].x = (p.x + q.x)/2;
            segments_original[0].y = (p.y + q.y)/2;

            if (Dm > square_width*S)
            {
                segments_original[0].type = visualization_msgs::Marker::SPHERE;
                segments_original[0].radius = S/2;
                segments.push_back(segments_original[0]);
                segments_original.erase(segments_original.begin());
            }
            else
            {
                segments_original[0].type = visualization_msgs::Marker::CUBE;
                segments_original[0].width = abs(q.x - p.x);
                segments_original[0].height = abs(q.y - p.y);
                segments.push_back(segments_original[0]);
                segments_original.erase(segments_original.begin());
            }
        }
        else
        {
            segments_original.erase(segments_original.begin());
        }


    }

}

void scan2dClass::__AssociateSegments(std::vector<SEGMENT> &segments)
{
	std::vector<SEGMENT> segments_global;
	for (const auto& segment : segments)
	{
		geometry_msgs::PoseStamped segment_scan;
		segment_scan.header = scan_.header;
		segment_scan.pose = potbot_lib::utility::get_Pose(segment.x, segment.y, 0,0,0,0);
		geometry_msgs::Pose segment_pose_global = potbot_lib::utility::get_tf(tf_buffer_, segment_scan, FRAME_ID_GLOBAL).pose;

		SEGMENT segment_global;
		segment_global.height = segment.height;
		segment_global.width = segment.width;
		segment_global.id = segment.id;
		segment_global.radius = segment.radius;
		segment_global.type = segment.type;
		segment_global.x = segment_pose_global.position.x;
		segment_global.y = segment_pose_global.position.y;

		segments_global.push_back(segment_global);
	}

    static std::vector<SEGMENT> segments_pre;
    static int global_idx = 0;
    for(int i = 0; i < segments_global.size(); i++)
    {
        double distance_min = std::numeric_limits<double>::infinity();
        int idx = 0;
        for(int j = 0; j < segments_pre.size(); j++)
        {
            double distance = sqrt(pow((segments_global[i].x - segments_pre[j].x),2) + 
                                    pow(segments_global[i].y - segments_pre[j].y,2));
            if(distance < distance_min)
            {
                distance_min = distance;
                idx = j;
            }
        }

        if (distance_min > 1)
        {
            segments_global[i].id = global_idx++;
        }
        else
        {
			if (distance_min > 0.03) 
			{
				segments_global[i].is_moving = true;
				segments[i].is_moving = true;
			}
            segments_global[i].id = segments_pre[idx].id;
			segments[i].id = segments_pre[idx].id;
        }
    }
    segments_pre = segments_global;
}