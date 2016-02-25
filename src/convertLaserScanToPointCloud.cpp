#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>

laser_geometry::LaserProjection projector_;
ros::Publisher  pub;

void scanCallback(const sensor_msgs::LaserScan scan)
{
  sensor_msgs::PointCloud::Ptr cloud(new sensor_msgs::PointCloud());
  cloud->header.frame_id = scan.header.frame_id;//"filtered_velodyne";
  cloud->header.stamp = scan.header.stamp;
  for (unsigned int i=0; i<scan.ranges.size();i++)
  {
    //ROS_INFO("1st scanning");
    //scan.angle_increment = 0.004363 rad = 0.25 deg 
    double ranges = scan.ranges[i];
    double x=ranges*sin(i*scan.angle_increment-180*scan.angle_increment);//+xtransform;//0.17; 
    double y=ranges*-cos(i*scan.angle_increment-180*scan.angle_increment);
    //"-180*scan.angle_increment" is because the blue laser reading starts from: -(1080-720)/2*0.004363
    //x[i]=ranges[i]*sin(i*scan.angle_increment-0*scan.angle_increment);//+xtransform;//0.17; 
    //y[i]=ranges[i]*-cos(i*scan.angle_increment-0*scan.angle_increment);

    //if(ranges[i]==0)
    //{
      ////ROS_INFO("No laser reading[%d]",i);   
    //}

    // laser reading 0-1039. middle point is 512, right point is 158, left point is 881
    // by experiments 163-256, 285-383, 436-618, 652-756, 780-870 is key in detecting obstales

    // Blue laser reading 0-1080, middle point is 540, right point is 180, left point is 900
    geometry_msgs::Point32 point_new;
    point_new.x = x;
    point_new.y = y;
    point_new.z = 0;
    cloud->points.push_back(point_new);
    pub.publish(cloud);
  }  
}
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
  //ros::Subscriber sub = nh.subscribe ("scan_base/scan", 1 , scanCallback);
  pub = nh.advertise<sensor_msgs::PointCloud>( "converted_laserscan", 1);

  while (ros::ok()){ros::spinOnce();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
  return 0;
}
