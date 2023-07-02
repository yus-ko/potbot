#include<potbot/Filter.h>

void FilterClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
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

void FilterClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles_ = msg;
    std::cout<<"-----------------"<<std::endl;
    for(int i =0; i < obstacles_.markers.size(); i++)
    {
        
        if (obstacles_.markers[i].ns == "segments_display")
        {
            double t_now = obstacles_.markers[i].header.stamp.toSec();
            // double t_now = ros::Time::now().toSec();
            double x = obstacles_.markers[i].pose.position.x;
            double y = obstacles_.markers[i].pose.position.y;
            // ROS_INFO("%f, %f",x,y);
            static double x_pre = -1000;
            static double y_pre = -1000;
            static double t_pre = -1000;
            if (t_pre > 0)
            {
                double dt = t_now-t_pre;
                int ny = 2;
                Eigen::VectorXd observed_data(ny);
                observed_data<< x, y;
                std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = states_ukf_[0].update(observed_data,dt);
                Eigen::MatrixXd xhat = std::get<0>(ans);
                Eigen::MatrixXd P = std::get<1>(ans);
                Eigen::MatrixXd K = std::get<2>(ans);

                potbot::State state_msg;
                state_msg.header = obstacles_.markers[i].header;

                state_msg.z.data.resize(ny);
                state_msg.xhat.data.resize(xhat.rows());
                state_msg.P.data.resize(P.rows()*P.cols());
                state_msg.K.data.resize(K.rows()*K.cols());

                matrixToDoubleArray(observed_data, state_msg.z);
                matrixToDoubleArray(xhat, state_msg.xhat);
                matrixToDoubleArray(P, state_msg.P);
                matrixToDoubleArray(K, state_msg.K);
                
                std::cout<<xhat.transpose()<<std::endl;
                pub_state_.publish(state_msg);
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
            }
            x_pre = x;
            y_pre = y;
            t_pre = t_now;
            
            break;

        }
    }
}

// void FilterClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
// {
//     obstacles_ = msg;
//     std::cout<<"-----------------"<<std::endl;
//     for(int i =0; i < obstacles_.markers.size(); i++)
//     {
        
//         if (obstacles_.markers[i].ns == "segments_display")
//         {
//             // double t_now = obstacles_.markers[i].header.stamp.toSec();
//             double t_now = ros::Time::now().toSec();
//             double x = obstacles_.markers[i].pose.position.x;
//             double y = obstacles_.markers[i].pose.position.y;
//             // ROS_INFO("%f, %f",x,y);
//             static double x_pre = -1000;
//             static double y_pre = -1000;
//             static double t_pre = -1000;
//             if (t_pre >= 0)
//             {
//                 double dt = t_now-t_pre;
//                 double vx = (x-x_pre)/dt;
//                 double vy = (y-y_pre)/dt;
//                 Eigen::MatrixXd data(4,1);
//                 data<< x, y, vx, vy;
//                 states_[0].input_data(data,t_now);
//                 states_[0].update();
//                 potbot::State state_msg;
//                 state_msg.header = obstacles_.markers[i].header;

//                 state_msg.z.data.resize(4);
//                 state_msg.xhat.data.resize(4);
//                 state_msg.K.data.resize(16);
//                 state_msg.P.data.resize(16);

//                 matrixToDoubleArray(states_[0].get_z(), state_msg.z);
//                 matrixToDoubleArray(states_[0].get_xhat(), state_msg.xhat);
//                 matrixToDoubleArray(states_[0].get_K(), state_msg.K);
//                 matrixToDoubleArray(states_[0].get_P(), state_msg.P);
                
//                 // state_msg.xhat = states_[0].get_xhat();
//                 // state_msg.K = states_[0].get_K();
//                 // state_msg.P = states_[0].get_P();

//                 // double vxhat = states_[0].get_state()(2,0);
//                 // double vyhat = states_[0].get_state()(3,0);
//                 // state_msg.vector.x = vxhat;
//                 // state_msg.vector.y = vyhat;
//                 // ROS_INFO("%f, %f",vy,vyhat);
//                 // ROS_INFO("%f, %f, %f",t_now, t_pre, dt);

//                 std::cout<<states_[0].get_xhat().transpose()<<std::endl;
//                 pub_state_.publish(state_msg);
//             }
//             x_pre = x;
//             y_pre = y;
//             t_pre = t_now;
            
//             break;

//         }
//     }
// }