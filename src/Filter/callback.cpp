#include<potbot/Filter.h>

void FilterClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
}

void FilterClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles_ = msg;
    // std::cout<<"-----------------"<<std::endl;
    for(int i =0; i < obstacles_.markers.size(); i++)
    {
        
        if (obstacles_.markers[i].ns == "segments_display")
        {
            double t_now = obstacles_.markers[i].header.stamp.toSec();
            double x = obstacles_.markers[i].pose.position.x;
            double y = obstacles_.markers[i].pose.position.y;
            // ROS_INFO("%f, %f",x,y);
            static double y_pre = -1000;
            static double t_pre = -1000;
            if (t_pre >= 0)
            {
                double dt = t_now-t_pre;
                double vy = (y-y_pre)/dt;
                Eigen::MatrixXd data(4,1);
                data(0,0) = x;
                data(1,0) = y;
                // data(3,0) = vy;
                states_[0].input_data(data,t_now);
                states_[0].update();
                geometry_msgs::Vector3Stamped state_msg;
                state_msg.header = obstacles_.markers[i].header;
                double vyhat = states_[0].get_state()(3,0);
                state_msg.vector.y = vyhat;
                // ROS_INFO("%f, %f",vy,vyhat);
                std::cout<<states_[0].get_state().transpose()<<std::endl;
                pub_state_.publish(state_msg);
            }
            y_pre = y;
            t_pre = t_now;
            
            break;

        }
    }
}