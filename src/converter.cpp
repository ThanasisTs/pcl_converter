// Convert a 3D pointcloud ROS message into a pcd file

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

std::string file;
std::string topic;

void convert(const sensor_msgs::PointCloud2ConstPtr& cloud_ROS){

  pcl::PCLPointCloud2* cloud_init = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_init);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA> ());

  pcl_conversions::toPCL(*cloud_ROS, *cloud_init);
  pcl::fromPCLPointCloud2(*cloud_init, *cloud);

  pcl::io::savePCDFileASCII(file, *cloud);
  ROS_INFO("File saved succefully");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "converter");
	ros::NodeHandle nh;
	nh.getParam("/converter/fileName", file);
	nh.getParam("/converter/topic", topic);
	ros::Subscriber sub = nh.subscribe(topic, 1, convert);		
	ros::spin();
}