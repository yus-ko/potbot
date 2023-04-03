//include haeders
#include <potbot/Utility.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//クラスの定義
class ControllerClass{

    private:
        
        //センサーデータ
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_odom_, sub_path_, sub_goal_, sub_local_map_, sub_scan_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_cmd_, pub_path_request_;

        tf::TransformListener tflistener;
        tf2_ros::Buffer tf_buffer_;

        int robot_id_ = MEGAROVER;
        
        geometry_msgs::Twist cmd_;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        nav_msgs::Odometry line_following_start_;
        nav_msgs::Path robot_path_;
        nav_msgs::OccupancyGrid local_map_;

        geometry_msgs::PoseStamped goal_;

        sensor_msgs::LaserScan scan_;

        int robot_path_index_ = 0;
        nav_msgs::Odometry robot_, odom_;
        std::string ROBOT_NAME;
        bool IS_SIMULATOR, PUBLISH_COMMAND;
        double PATH_TRACKING_MARGIN;

        void __odom_callback(const nav_msgs::Odometry& msg);
        void __goal_callback(const geometry_msgs::PoseStamped& msg);
        void __local_map_callback(const nav_msgs::OccupancyGrid& msg);
        void __scan_callback(const sensor_msgs::LaserScan& msg);
        void __publish_path_request();
        void __publishcmd();

        void __LineFollowing();
        void __PoseAlignment();
        bool __PathCollision();

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        ControllerClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~ControllerClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void path_callback(const nav_msgs::Path& msg);
        
        void mainloop();

        void manage();
        void controller();
        void calculate_cmd();
        
};
