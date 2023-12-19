#include<potbot_filter/Filter.h>

void FilterClass::mainloop()
{
    ros::spin();
    ros::Rate loop_rate(50);
    // KalmanFilter obs;
    // states_.push_back(obs);
	// while (ros::ok())
	// {
    //     manage();
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
}

void FilterClass::manage()
{
    filter();
}

void FilterClass::filter()
{
    //states_[0].update();
    //std::cout<<states_[0].get_state()<<std::endl;
}

double FilterClass::__Median(std::vector<double> v)
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

void FilterClass::__MedianFilter(sensor_msgs::LaserScan &scan)
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

void FilterClass::__Segmentation(sensor_msgs::LaserScan &scan, std::vector<SEGMENT> &segments)
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

double FilterClass::__distanceToLineSegment(POINT o, POINT p, POINT q)
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

void FilterClass::__SplitSegments(std::vector<SEGMENT> &segments)
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

void FilterClass::__AssociateSegments(std::vector<SEGMENT> &segments)
{
    static std::vector<SEGMENT> segments_pre;
    static int global_idx = 0;
    for(int i = 0; i < segments.size(); i++)
    {
        double distance_min = std::numeric_limits<double>::infinity();
        int idx = 0;
        for(int j = 0; j < segments_pre.size(); j++)
        {
            double distance = sqrt(pow((segments[i].x - segments_pre[j].x),2) + 
                                    pow(segments[i].y - segments_pre[j].y,2));
            if(distance < distance_min)
            {
                distance_min = distance;
                idx = j;
            }
        }

        if (distance_min > 1)
        {
            segments[i].id = global_idx++;
        }
        else
        {
            segments[i].id = segments_pre[idx].id;
            // segments_pre.erase(segments_pre.begin() + idx);
        }
    }
    segments_pre = segments;
}