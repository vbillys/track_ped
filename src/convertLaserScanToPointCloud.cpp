#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector_;
ros::Publisher  pub;

void convertScan (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*scan_in, cloud);
  pub.publish(cloud);
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "convert_laserscan_to_pointcloud_node");
  ros::NodeHandle nh;//("~");
  ros::Subscriber sub = nh.subscribe ("scan_base/scan", 1 , convertScan);
  pub = nh.advertise<sensor_msgs::PointCloud>( "converted_laserscan", 1);

  while (ros::ok()){ros::spinOnce();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
  return 0;
}
