#include <potbot_lib/Utility.h>
#include <potbot_msgs/ObstacleArray.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

//クラスの定義
class LocalmapClass{

    private:
        
        //センサーデータ
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_obstacles_scan_, sub_obstacles_pcl_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_localmap_;

        tf2_ros::TransformBroadcaster broadcaster_;

        potbot_msgs::ObstacleArray obstacles_scan_, obstacles_pcl_;

        // dynamic_reconfigure::Server<potbot_localization::LocalizationConfig> server_;
  	    // dynamic_reconfigure::Server<potbot_localization::LocalizationConfig>::CallbackType f_;

        void __obstacles_scan_callback(const potbot_msgs::ObstacleArray& msg);
        void __obstacles_pcl_callback(const potbot_msgs::ObstacleArray& msg);

        // void __param_callback(const potbot_localization::LocalizationConfig& param, uint32_t level);
        void __get_param();
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        LocalmapClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~LocalmapClass();

        
};
