#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Point 
{
    double x, y, z;
    Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// 球面座標を直交座標に変換する関数
Point sphericalToCartesian(double r, double theta, double phi) 
{
    double x = r * sin(theta) * cos(phi);
    double y = r * sin(theta) * sin(phi);
    double z = r * cos(theta);
    return Point(x, y, z);
}

void create_points(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	size_t numPoints = 10000; // 点の数
	// 球面座標系で点を生成して配列に格納
	for (int i = 0; i < numPoints; ++i) {
		double theta = acos(-1.0 + (2.0 * i) / numPoints); // θは[0, π]の範囲になる
		double phi = sqrt(numPoints * M_PI) * theta; // φは[0, 2π]の範囲になる
		Point p = sphericalToCartesian(sin(time), theta, phi);
		// p.x += sin(time);
		// p.z += cos(time);
		points_output.push_back(p);
	}
}

void pointvec_to_pcl(const std::vector<Point>& points_input, pcl::PointCloud<pcl::PointXYZ>& pcl_output)
{
	// pclのPointCloudを作成
	pcl::PointCloud<pcl::PointXYZ> pcl_init;
	pcl_output = pcl_init;
	pcl_output.width = points_input.size();
	pcl_output.height = 1; // 単一の点群
	pcl_output.points.resize(pcl_output.width * pcl_output.height);

	// 点のベクトルをPointCloudにコピー
	for (size_t i = 0; i < points_input.size(); ++i) {
		pcl_output.points[i].x = points_input[i].x;
		pcl_output.points[i].y = points_input[i].y;
		pcl_output.points[i].z = points_input[i].z;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_figure");
	ros::NodeHandle nh;
	ros::Publisher pub_pc = nh.advertise<sensor_msgs::PointCloud2>("pcl_figure", 1);
	ros::Rate loop_rate(30);

	while (ros::ok())
	{

		std::vector<Point> points;
		create_points(points, ros::Time::now().toSec());

		pcl::PointCloud<pcl::PointXYZ> pcl;
		pointvec_to_pcl(points, pcl);

		// sensor_msgs::PointCloud2に変換
		sensor_msgs::PointCloud2 pcl_msg;
		pcl::toROSMsg(pcl, pcl_msg);
		pcl_msg.header.frame_id = "map"; // フレームIDを設定

		pub_pc.publish(pcl_msg);
		loop_rate.sleep(); // ループの周期を調整
	}
	

    

    return 0;
}
