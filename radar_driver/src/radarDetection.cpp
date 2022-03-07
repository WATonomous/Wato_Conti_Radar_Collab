#include <ros/ros.h>
#include <string>

 //msgs
 #include "radar_driver/RadarPacket.h"
 #include "multiRadar.h"
 #include "sensor_msgs/PointCloud2.h"
 #include <mutex>

extern int opterr;

MultiRadar::MultiRadar(ros::NodeHandle &nh){
  // std::string right_topic("/radar_right/filtered_radar_packet");
  // std::string left_topic("/radar_left/filtered_radar_packet");
  // std::string right_topic("/carla/ego_vehicle/radar/left_radar/radar_points");
  // std::string left_topic("/carla/ego_vehicle/radar/right_radar/radar_points");
  // int q = 10; //queue size

  // right_radar_sub = nh.subscribe(right_topic, q, &MultiRadar::rightDetection, this);
  // left_radar_sub = nh.subscribe(left_topic, q, &MultiRadar::leftDetection, this);

  // radar_timer = nh.createTimer(ros::Duration(1), &MultiRadar::detection, this);
}

void MultiRadar::detection(const ros::TimerEvent &event){
  // if(right_radar_detection == true && left_radar_detection == true){
  //   ROS_WARN_STREAM("Both radars detecting object...");
  //   /*If the right radar detects a point first, then the left
  //   radar should drop the point.*/
  //   if(right_radar.TimeStamp < left_radar.TimeStamp){
  //     ROS_WARN_STREAM("Dropping left radar point...");
  //     left_radar_detection = false;
  //     //This is just a message, haven't done the actual dropping yet
  //   }
  //   else{
  //     ROS_WARN_STREAM("Dropping right radar point...");
  //     right_radar_detection = false;
  //   }
  // }
}

void MultiRadar::rightDetection(const radar_driver::RadarPacket::ConstPtr &right_radar_msg){
  // if(right_radar_msg->Detections.size() > 1){
  //   right_radar_detection = true;
  //   right_radar = *right_radar_msg;
  //   ROS_INFO_STREAM("Right Radar Detection");
  // }
  // else{
  //   right_radar_detection = false;
  //   right_radar = *right_radar_msg;
  // }
}

void MultiRadar::leftDetection(const radar_driver::RadarPacket::ConstPtr &left_radar_msg){
  // if(left_radar_msg->Detections.size() > 1){
  //   left_radar_detection = true;
  //   left_radar = *left_radar_msg;
  //   ROS_INFO_STREAM("Left Radar Detection");
  // }
  // else{
  //   left_radar_detection = false;
  //   left_radar = *left_radar_msg;
  // }
}

int main(int argc, char** argv)
{
  // opterr = 0;

  // ros::init(argc, argv, "radar_filter");

  // ros::NodeHandle nh;
  // MultiRadar multiradar(nh);

  // ros::Publisher overlap_right = nh.advertise<sensor_msgs::PointCloud2>("/radar_right/radar_pointcloud", 50);
  // ros::Publisher overlap_left = nh.advertise<sensor_msgs::PointCloud2>("/radar_left/radar_pointcloud", 50);

  // ros::spin();

  return 0;
}
