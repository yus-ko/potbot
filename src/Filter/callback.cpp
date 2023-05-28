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

void FilterClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles_ = msg;
    std::cout<<"-----------------"<<std::endl;
    for(int i =0; i < obstacles_.markers.size(); i++)
    {
        
        if (obstacles_.markers[i].ns == "segments_display")
        {
            // double t_now = obstacles_.markers[i].header.stamp.toSec();
            double t_now = ros::Time::now().toSec();
            double x = obstacles_.markers[i].pose.position.x;
            double y = obstacles_.markers[i].pose.position.y;
            // ROS_INFO("%f, %f",x,y);
            static double x_pre = -1000;
            static double y_pre = -1000;
            static double t_pre = -1000;
            if (t_pre >= 0)
            {
                double dt = t_now-t_pre;
                double vx = (x-x_pre)/dt;
                double vy = (y-y_pre)/dt;
                Eigen::MatrixXd data(4,1);
                data<< x, y, vx, vy;
                states_[0].input_data(data,t_now);
                states_[0].update();
                potbot::State state_msg;
                state_msg.header = obstacles_.markers[i].header;

                state_msg.z.data.resize(4);
                state_msg.xhat.data.resize(4);
                state_msg.K.data.resize(16);
                state_msg.P.data.resize(16);

                matrixToDoubleArray(states_[0].get_z(), state_msg.z);
                matrixToDoubleArray(states_[0].get_xhat(), state_msg.xhat);
                matrixToDoubleArray(states_[0].get_K(), state_msg.K);
                matrixToDoubleArray(states_[0].get_P(), state_msg.P);
                
                // state_msg.xhat = states_[0].get_xhat();
                // state_msg.K = states_[0].get_K();
                // state_msg.P = states_[0].get_P();

                // double vxhat = states_[0].get_state()(2,0);
                // double vyhat = states_[0].get_state()(3,0);
                // state_msg.vector.x = vxhat;
                // state_msg.vector.y = vyhat;
                // ROS_INFO("%f, %f",vy,vyhat);
                // ROS_INFO("%f, %f, %f",t_now, t_pre, dt);
                
                std::cout<<states_[0].get_xhat().transpose()<<std::endl;
                pub_state_.publish(state_msg);
            }
            x_pre = x;
            y_pre = y;
            t_pre = t_now;
            
            break;

        }
    }
}