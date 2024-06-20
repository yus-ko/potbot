#include<potbot_filter/Filter.h>

#define __NX__ 5
#define __NY__ 2

namespace potbot_filter
{

    FilterClass::FilterClass()
    {
        
        ros::NodeHandle n("~");
        //n.getParam("",);
        n.getParam("sigma_p",               sigma_p_);
        n.getParam("sigma_q",               sigma_q_);
        n.getParam("sigma_r",               sigma_r_);

        sub_obstacle_				= nhSub_.subscribe("obstacle/scan/clustering",			1,&FilterClass::__obstacle_callback,this);
        
        pub_state_					= nhPub_.advertise<potbot_msgs::StateArray>(			"state", 1);
        pub_state_markers_			= nhPub_.advertise<visualization_msgs::MarkerArray>(	"state/marker", 1);
        pub_obstacles_pcl_			= nhPub_.advertise<potbot_msgs::ObstacleArray>(			"obstacle/pcl/estimate", 1);
        pub_obstacles_scan_			= nhPub_.advertise<potbot_msgs::ObstacleArray>(			"obstacle/scan/estimate", 1);
    }
    FilterClass::~FilterClass(){
    }

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
    
    void FilterClass::__obstacle_callback(const potbot_msgs::ObstacleArray& msg)
    {
        potbot_msgs::ObstacleArray obstacle_array = msg;
        static std::vector<int> ukf_id;
        
        if (obstacle_array.data.empty())
        {
            pub_obstacles_scan_.publish(obstacle_array);
            return;
        }

        double t_now = obstacle_array.header.stamp.toSec();
        // double t_now = ros::Time::now().toSec();
        static double t_pre = t_now;
        double dt = t_now-t_pre;

        potbot_msgs::StateArray state_array_msg;
        for(auto& obstacle : obstacle_array.data)
        {
            if (!obstacle.is_moving) continue;
            
            double x = obstacle.pose.position.x;
            double y = obstacle.pose.position.y;
            int id = obstacle.id;

            auto iter = std::find(ukf_id.begin(), ukf_id.end(), id);
            if (iter != ukf_id.end())
            {
                int index_ukf = std::distance(ukf_id.begin(), iter);
                int ny = 2;
                Eigen::VectorXd observed_data(ny);
                observed_data<< x, y;
                std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = states_ukf_[index_ukf].update(observed_data,dt);
                Eigen::VectorXd xhat = std::get<0>(ans);
                Eigen::MatrixXd P = std::get<1>(ans);
                Eigen::MatrixXd K = std::get<2>(ans);

                potbot_msgs::State state_msg;
                state_msg.header = obstacle.header;
                state_msg.id = id;

                state_msg.state.resize(4);

                state_msg.state[0].label = "z";
                state_msg.state[0].matrix = potbot_lib::utility::matrix_to_multiarray(observed_data);

                state_msg.state[1].label = "xhat";
                state_msg.state[1].matrix = potbot_lib::utility::matrix_to_multiarray(xhat);

                state_msg.state[2].label = "P";
                state_msg.state[2].matrix = potbot_lib::utility::matrix_to_multiarray(P);

                state_msg.state[3].label = "K";
                state_msg.state[3].matrix = potbot_lib::utility::matrix_to_multiarray(K);
                
                //std::cout<<xhat.transpose()<<std::endl;
                state_array_msg.data.push_back(state_msg);

                obstacle.pose.position.x = xhat(0);
                obstacle.pose.position.y = xhat(1);
                obstacle.pose.orientation  = potbot_lib::utility::get_Quat(0,0,xhat(2));
                obstacle.twist.linear.x = xhat(3);
                obstacle.twist.angular.z = xhat(4);
            }
            else
            {
                
                Eigen::MatrixXd Q(__NY__,__NY__), R(__NX__,__NX__), P(__NX__,__NX__);
                Q.setZero();R.setZero();P.setZero();
                // for (int i = 0; i < __NY__; i++) Q(i,i) = sigma_q_;
                // for (int i = 0; i < __NX__; i++) R(i,i) = sigma_r_;
                // for (int i = 0; i < __NX__; i++) P(i,i) = sigma_p_;

                Q<< sigma_q_, 0,
                    0, sigma_q_;
                
                R<< 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, sigma_r_, 0,
                    0, 0, 0, 0, sigma_r_;

                P<< sigma_p_, 0, 0, 0, 0,
                    0, sigma_p_, 0, 0, 0,
                    0, 0, sigma_p_, 0, 0,
                    0, 0, 0, sigma_p_, 0,
                    0, 0, 0, 0, sigma_p_;

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
                Eigen::VectorXd xhat = potbot_lib::utility::multiarray_to_matrix(state_msg.state[1].matrix);
                if (abs(xhat(3)) > 2.0 || abs(xhat(3)) < 0.1) continue;

                visualization_msgs::Marker state_marker;

                state_marker.header             = state_msg.header;

                state_marker.ns                 = "segments/centor";
                state_marker.id                 = state_msg.id;
                state_marker.lifetime           = ros::Duration(1);

                state_marker.type               = visualization_msgs::Marker::ARROW;
                state_marker.action             = visualization_msgs::Marker::MODIFY;

                state_marker.pose.position.x    = xhat(0);
                state_marker.pose.position.y    = xhat(1);
                state_marker.pose.position.z    = 0;

                state_marker.pose.orientation   = potbot_lib::utility::get_Quat(0,0,xhat(2));

                state_marker.scale.x            = 0.05;
                state_marker.scale.y            = 0.1;
                state_marker.scale.z            = 0.1;
                
                state_marker.color              = potbot_lib::color::get_msg(state_msg.id);
                state_marker.color.a            = 0.5;

                geometry_msgs::Point p0, p1;
                p0.x                            = 0;
                p0.y                            = 0;
                p0.z                            = 0;
                p1.x                            = xhat(3);
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
}