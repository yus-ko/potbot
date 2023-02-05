//include haeders
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//クラスの定義
class RobotControlClass{

    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub_encoder;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_cmd, pub_odom;

        ros::Time encoder_time_pre;
        ros::WallTime start_time;
        geometry_msgs::Twist encoder_value, cmd;
        nav_msgs::Odometry odom;
        bool encoder_firsttime = true;

        bool done_turn = false, done_straight = false;

        double error_pos_x_pre = 0.0, error_pos_y_pre = 0.0, error_vel_pre = 0.0, error_angular_pre = 0.0, integral_vel_error = 0.0, integral_angular_error = 0.0;
        double encoder_deltatime, target_angle;


        bool IS_SIMULATOR, PUBLISH_COMMAND, ANGLE_CORRECTION, PID_CONTROL;
        double MAX_VELOCITY, MAX_ANGULAR, TARGET_POSITION_X, TARGET_POSITION_Y;
        double GAIN_PROPORTIONAL, GAIN_INTEGRAL, GAIN_DIFFERENTIAL;

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        RobotControlClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~RobotControlClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
	    void encoder_callback(const geometry_msgs::Twist& msg);
        void encoder_callback_sim(const nav_msgs::Odometry& msg);

        void manage();

        void odometry();
        void pid_control();

        //センサデータ送信
        void publishcmd();//データ送信
        void publishodom();//データ送信
        //データクリア
        //void clearMessages();
};
