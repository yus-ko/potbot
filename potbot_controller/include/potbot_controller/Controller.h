//include haeders
#include <potbot_lib/Utility.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_controller/ControllerConfig.h>

//クラスの定義
class ControllerClass{

    private:
        
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_odom_, sub_path_, sub_goal_;
        
        ros::NodeHandle nhPub_;
		ros::Publisher pub_cmd_, pub_path_request_, pub_look_ahead_;

        tf2_ros::Buffer tf_buffer_;
        
        geometry_msgs::Twist cmd_;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        nav_msgs::Odometry line_following_start_;
        nav_msgs::Path robot_path_;

        bool done_init_pose_alignment_;

        geometry_msgs::PoseStamped goal_;

        dynamic_reconfigure::Server<potbot_controller::ControllerConfig> server_;
  	    dynamic_reconfigure::Server<potbot_controller::ControllerConfig>::CallbackType f_;

        int robot_path_index_ = 0;
        nav_msgs::Odometry robot_, odom_;
        std::string FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, TOPIC_ODOM, TOPIC_CMD_VEL;
        bool PUBLISH_COMMAND;
        double PATH_TRACKING_MARGIN, TARGET_POSITION_X, TARGET_POSITION_Y, TARGET_POSITION_YAW, MAX_LINEAR_VELOCITY = 0.2, MAX_ANGULAR_VELOCITY = 1.0;

        void __odom_callback(const nav_msgs::Odometry& msg);
        void __goal_callback(const geometry_msgs::PoseStamped& msg);
        void __path_callback(const nav_msgs::Path& msg);
        void __param_callback(const potbot_controller::ControllerConfig& param, uint32_t level);
        void __publish_path_request();
        void __publishcmd();

        void __LineFollowing();
        void __PoseAlignment(geometry_msgs::Pose target);

        void __get_param();

    public:
        ControllerClass();
        ~ControllerClass();
        
        void mainloop();

        void manage();
        void controller();
        
};
