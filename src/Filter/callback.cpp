#include<potbot/Filter.h>

void FilterClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
}

void FilterClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles_ = msg;
    for(int i =0; i < obstacles_.markers.size(); i++)
    {
        if (obstacles_.markers[i].ns == "segments_display")
        {
            
        }
    }
}