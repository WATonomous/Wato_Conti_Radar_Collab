#include "radarPublisher.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>

//msgs
#include "std_msgs/String.h"
#include "radar_driver/EthCfg.h"
#include "radar_driver/RadarDetection.h"
#include "radar_driver/RadarPacket.h"
#include "radar_driver/SensorCfg.h"
#include "radar_driver/SensorStatus.h"

//#define PRINT_PUBLISHER

RadarPublisher::RadarPublisher (ros::NodeHandle nh_)
{
  packet_pub_  = nh_.advertise<radar_driver::RadarPacket>(std::string("filtered_radar_packet"), 50);
}

void RadarPublisher::publishRadarPacketMsg(radar_driver::RadarPacket& radar_packet_msg)
{
  return packet_pub_.publish(radar_packet_msg);
}

void RadarPublisher::publishRadarDetectionMsg(radar_driver::RadarDetection& radar_detection_msg)
{
  return detection_pub_.publish(radar_detection_msg);
}

uint8_t RadarPublisher::pubCallback(PacketGroup_t* Packets) {   //call upon the appropriate publish function
  for (uint8_t i = 0; i < Packets->numFarPackets; i++) {
    this->publishRadarPacketMsg(Packets->farPackets[i]);
    ROS_INFO_STREAM("Far packet timestamp: " << std::to_string((Packets->farPackets[i]).TimeStamp));
  }

  for (uint8_t i = 0; i < Packets->numNearPackets; i++) {
    this->publishRadarPacketMsg(Packets->nearPackets[i]);
    ROS_INFO_STREAM("Near packet timestamp: " << std::to_string((Packets->nearPackets[i]).TimeStamp));
  }

  return SUCCESS;
}
