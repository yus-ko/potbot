#ifndef _H_SCANCLUSTERING_
#define _H_SCANCLUSTERING_

#include <potbot_lib/Utility.h>
#include <potbot_msgs/ObstacleArray.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

namespace potbot_lib{

    typedef struct {
        int index=0;
        double x=0;
        double y=0;
        double r=0;
        double theta=0;
    } POINT;

    typedef struct {
        std::vector<POINT> points;
        int id=0;
        int type=0;
        double x=0;
        double y=0;
        double radius=0;
        double width=0;
        double height=0;
        bool is_moving=false;
    } SEGMENT;

    class ScanClustering{
        private:
            std::vector<SEGMENT> clusters_;

        public:
            ScanClustering();
            ~ScanClustering(){};
            
            void set_scan(const sensor_msgs::LaserScan& scan);
            void get_clusters(std::vector<SEGMENT>& clusters_arg);
            void euclidean_clustering();
            void segmentation();
            void to_markerarray(visualization_msgs::MarkerArray& ma);
            void to_obstaclearray(potbot_msgs::ObstacleArray& oa);

    };
}

#endif	// _H_SCANCLUSTERING_