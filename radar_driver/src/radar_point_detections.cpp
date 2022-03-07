#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <string>
#include <mutex>

extern int opterr;

void rightDetection(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud_msg, "x"); it != it.end(); ++it) {
        ROS_INFO_STREAM("Right radar: " << it[0] << ", " << it[1] << ", " << it[2]);
  }
}

void leftDetection(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud_msg, "x"); it != it.end(); ++it) {
        ROS_INFO_STREAM("Left radar: " << it[0] << ", " << it[1] << ", " << it[2]);
  }
}


int main(int argc, char** argv)
{
  opterr = 0;

  ros::init(argc, argv, "radar_points");
  ros::NodeHandle nh;
  
  std::string right_topic("/carla/ego_vehicle/radar/right_radar/radar_points");
  std::string left_topic("/carla/ego_vehicle/radar/left_radar/radar_points");
  int q = 10; //queue size

  ros::Subscriber right_radar_sub = nh.subscribe(right_topic, q, rightDetection);
  ros::Subscriber left_radar_sub = nh.subscribe(left_topic, q, leftDetection);

  ros::spin();

  return 0;
}
