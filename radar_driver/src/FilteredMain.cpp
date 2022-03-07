#include <ros/ros.h>
#include <getopt.h>
#include "radarPublisher.h"
#include "processPacket.h"

extern int opterr;

PacketProcessor packetProcessor; 

void printUsage(char buff[]) {
  printf("\nUsage: %s [-h] [-n RADARNAME][-y UNFILTEREDTOPIC]\r\n", buff);
  printf("\tn: radar namespace (defaults: radar), should use launch script for this\r\n");
  printf("\tt: unfiltered topic name if reading (defaults: unfiltered_radar_packet\r\n");
}

void radarCallback(const radar_driver::RadarPacket::ConstPtr& msg) {
  uint8_t ret_status = packetProcessor.processRDIMsg(msg);
  std::string status_info = "";

  // Most of our return statuses are of type ERR so make this the default verbosity level
  uint8_t log_verbosity = LOG_ERROR;

  switch(ret_status) {
    case NO_DETECTIONS:
      status_info = "No Detections";
      // No detections isn't really a failure, so just issue a warning so that it gets logged somewhere
      log_verbosity = LOG_WARNING;
      break;
    case BAD_EVENT_ID:
      status_info = "Bad Event ID";
      break;
    case NO_PUB_CLR_FAIL:
      status_info = "No Publisher and Unable to Clear Packets";
      break;
    case NO_PUBLISHER:
      status_info = "No Publisher";
      break;
    case PUBLISH_FAIL:
      status_info = "Publishing failed";
      break;
    case CLEAR_FAIL:
      status_info = "Failed to clear packets";
      break;
    default:
      // Assume default ret_value is success in which case don't log anything
      return;
  }

  if (log_verbosity == LOG_WARNING) {
    ROS_WARN_STREAM("radarCallback warning, unable to execute processRDIMsg. Cause: " << status_info);
  } else if (log_verbosity == LOG_ERROR) {
    ROS_ERROR_STREAM("radarCallback ERROR, unable to execute processRDIMsg. Cause: " << status_info);
  }
}

int main(int argc, char** argv)
{
  opterr = 0;

  ros::init(argc, argv, "radar_processor");
  ROS_INFO("Initialize radar processor");
  int c;
  uint8_t scanMode = 0;
  std::string unfiltered_topic = "unfiltered_radar_packet";
  std::string radar_name = "";
  // Get the command line option, if any
  while ((c = getopt (argc, argv, "+ht:s:")) != -1)
  { //Pass remainder of arguments to the sniffer
    switch (c)
    {
      case 'h':
        printUsage(argv[0]);
        exit(0);
        break;
      case 't':
        unfiltered_topic = std::string(optarg);
        printf("Unfiltered topic: %s\r\n", unfiltered_topic);
        break;
      case 's':
        scanMode = atoi(optarg);
        printf("scanMode: %d\r\n", scanMode);
        break;
      case 'n':
        radar_name = std::string(optarg);
        break;
    }
  }

  if (argc <= 1) {
      ROS_INFO("Insufficient args\r\n");
      printUsage(argv[0]);
      return 0;
  }

  if (scanMode > 2) {
      ROS_INFO("Scan Mode must be 0,1,2\r\n");
      printUsage(argv[0]);
      return 0;
  }

  ros::NodeHandle nh;
  RadarPublisher rp(nh);
  ros::Subscriber radarUnfilteredSub;

  if (!packetProcessor.initializePacketProcessor(&rp, scanMode)) {
    radarUnfilteredSub = nh.subscribe(unfiltered_topic, 100, radarCallback);
    ros::spin();
  } else {
    ROS_INFO("Failed to setup processor\r\n");
  }

  return 0;
}
