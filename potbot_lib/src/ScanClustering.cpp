#include <potbot_lib/ScanClustering.h>

namespace potbot_lib{

    ScanClustering::ScanClustering()
    {

    }

    void ScanClustering::set_scan(const sensor_msgs::LaserScan& scan)
    {
        clusters_.clear();
        SEGMENT clus;
        size_t p_idx = 0, range_idx = 0;
        for (const auto& range : scan.ranges)
        {
            if (scan.range_min <= range && range <= scan.range_max)
            {
                POINT p;
                p.index = p_idx++;
                p.theta = double(range_idx++ * scan.angle_increment) + scan.angle_min;
                p.r = range + scan.range_min;   //minは足さない
                p.x = p.r * cos(p.theta);
                p.y = p.r * sin(p.theta);
                clus.points.push_back(p);
            }
        }
        clusters_.push_back(clus);
        
    }
    
    void ScanClustering::get_clusters(std::vector<SEGMENT>& clusters_arg)
    {
        clusters_arg = clusters_;
    }

    void ScanClustering::segmentation()
    {

    }

    // void ScanClustering::euclidean_clustering()
    // {
    //     size_t size = clusters_[0].points.size();
    //     std::vector<bool> checked(size);
    //     std::fill(checked.begin(), checked.end(), false);

    //     for(size_t i = 0; i < clusters_[0].points.size(); i++)
    //     {
    //         const POINT& pi = clusters_[0].points[i];
    //         for(size_t j = 0; j < clusters_[0].points.size(); j++)
    //         {
    //             if (checked[j]) continue;
    //             const POINT& pj = clusters_[0].points[j];
    //             double distance = sqrt(pow(pi.x - pj.x,2) + pow(pi.y - pj.y,2));
    //             if (distance <= 0.3)
    //             {

    //             }
    //         }

    //         checked[i] = true;
    //     }

    // }

    void ScanClustering::euclidean_clustering()
    {
        int size = clusters_[0].points.size();
        bool start = false;
        SEGMENT seg;
        for (int i = 0; i < size; i++)
        {
            
            if(!start)
            {
                start = true;
                seg.points.clear();
            }

            POINT &p = clusters_[0].points[i];

            if (!seg.points.empty())
            {    
                POINT &p_next = seg.points.back();

                double distance = sqrt(pow(p.x - p_next.x,2) + pow(p.y - p_next.y,2));
                if (distance <= 0.3)
                {
                    seg.points.push_back(p);
                }
                else
                {
                    start = false;
                    clusters_.push_back(seg);
                    continue;
                }
            }
            else
            {
                seg.points.push_back(p);
            }

            if (i == size - 1 && start)
            {
                clusters_.push_back(seg);
            }
        }

        clusters_.erase(clusters_.begin());

        int id = 0;
        for (auto& clus : clusters_)
        {

            std::vector<double> vec_x, vec_y;
            //x のみを抽出
            std::transform(clus.points.begin(), clus.points.end(), std::back_inserter(vec_x), [](const POINT& p) { return p.x; });
            //y のみを抽出
            std::transform(clus.points.begin(), clus.points.end(), std::back_inserter(vec_y), [](const POINT& p) { return p.y; });

            // xの最小値を取得
            auto min_x_itr = std::min_element(vec_x.begin(), vec_x.end());
            double min_x = (min_x_itr != vec_x.end()) ? *min_x_itr : 0.0;
            // xの最大値を取得
            auto max_x_itr = std::max_element(vec_x.begin(), vec_x.end());
            double max_x = (max_x_itr != vec_x.end()) ? *max_x_itr : 0.0;

            // yの最小値を取得
            auto min_y_itr = std::min_element(vec_y.begin(), vec_y.end());
            double min_y = (min_y_itr != vec_y.end()) ? *min_y_itr : 0.0;
            // yの最大値を取得
            auto max_y_itr = std::max_element(vec_y.begin(), vec_y.end());
            double max_y = (max_y_itr != vec_y.end()) ? *max_y_itr : 0.0;

            clus.id = id++;
            clus.type = visualization_msgs::Marker::SPHERE;
            clus.width = abs(max_x - min_x);
            clus.height = abs(max_y - min_y);
            clus.x = min_x + clus.width/2.0;
            clus.y = min_y + clus.height/2.0;
        }

    }

    void ScanClustering::to_markerarray(visualization_msgs::MarkerArray& ma)
    {
        ma.markers.clear();
        int points_id = clusters_.back().id + 1;
        for (const auto& clus : clusters_)
        {
            visualization_msgs::Marker marker;

            marker.ns = "scan2d/centor";
            marker.id = clus.id;
            marker.lifetime = ros::Duration(1);

            marker.type = clus.type;
            marker.action = visualization_msgs::Marker::MODIFY;

            marker.pose = potbot_lib::utility::get_Pose(clus.x, clus.y, 0,0,0,0);

            marker.scale.x = clus.width;
            marker.scale.y = clus.height;
            marker.scale.z = 0.001;

            marker.color = potbot_lib::color::get_msg(clus.id);
            marker.color.a = 0.3;
            
            ma.markers.push_back(marker);

            visualization_msgs::Marker points;
            points.ns = "scan2d/points";
            points.id = points_id++;
            points.type = visualization_msgs::Marker::SPHERE_LIST;
            points.action = visualization_msgs::Marker::ADD;
            points.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);
            points.scale.x = 0.01;
            points.scale.y = 0.01;
            points.scale.z = 0.01;
            points.color = marker.color;

            for (const auto& point : clus.points)
            {
                points.points.push_back(potbot_lib::utility::get_Point(point.x, point.y, 0));
                points.colors.push_back(points.color);
            }
            ma.markers.push_back(points);
        }
    }

}
