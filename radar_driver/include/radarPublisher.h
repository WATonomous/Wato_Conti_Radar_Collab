#ifndef RADAR_PUBLISHER_H
#define RADAR_PUBLISHER_H

/* File: radarPublisher.h
 *
 * Publishes the radar data
 */

#include <ros/ros.h>
#include <sys/types.h>
#include <sys/prctl.h>
#include "processPacket.h"

 //msgs
 #include "radar_driver/RadarPacket.h"
 #include "radar_driver/RadarDetection.h"
 #include "radar_driver/EthCfg.h"
 #include "radar_driver/SensorCfg.h"
 #include "radar_driver/SensorStatus.h"

class RadarPublisher {
	private:
		ros::NodeHandle nh_; //Node Handle

		// Publisher Objects
		ros::Publisher packet_pub_;
		ros::Publisher detection_pub_; //Not used now, kept if useful

		//Publisher Functions
		void publishRadarPacketMsg(radar_driver::RadarPacket& radar_packet_msg);
		void publishRadarDetectionMsg(radar_driver::RadarDetection& radar_detection_msg);

	public:
		RadarPublisher(ros::NodeHandle new_nh_);
		uint8_t pubCallback(PacketGroup_t* Packets);
};

#endif
