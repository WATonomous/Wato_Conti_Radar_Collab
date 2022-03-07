#include <ros/ros.h>
#include <getopt.h>
#include "parser.h"

extern int opterr;

void printUsage(char buff[]) {
  printf("\nUsage: %s [-h] [-e interface] [-n packets] [-p port] [-f filter] [-c capture] [-l cpath] \r\n", buff);
  printf("\te: ethernet interface (string)\n");
  printf("\tn: number of packets (int)\n");
  printf("\tp: port (int)\n");
  printf("\tf: extra filter string (optional), enclose string in \" \" (string)\n");
  printf("\tc: capture method (int), default 1, LIVE\n");
  printf("\tl: file path to captured data (string), required for OFFLINE capture\n");
}

int main(int argc, char** argv)
{
  opterr = 0;
  ros::init(argc, argv, "radar_publisher");
  ROS_INFO("Initialize radar parser");
  char interface[16] = ""; //Which network interface (combination of port & IP) are we reading from?
  char filter[256] = ""; //What simple packet filters are we applying?
  int packets = 0, c, port = 31122; //Radar Default
  int capture_live = 1; //Based on if we're using live capture or from offline pcap doc
  char capture_path[256]  = ""; //File path to pcap doc if using offline capture

  // Get the command line option, if any
  while ((c = getopt (argc, argv, "+hp:e:n:c:l:f:")) != -1)
  { //Pass remainder of arguments to the sniffer
    switch (c)
    {
      case 'h':
        printUsage(argv[0]);
        exit(0);
        break;
      case 'e':
        strcpy(interface, optarg);
        printf("INTERFACE: %s\r\n", optarg);
        break;
      case 'n':
        packets = atoi(optarg);
        break;
      case 'p':
        port = atoi(optarg);
        printf("PORT: %d\r\n", port);
        break;
      case 'c':
        capture_live = atoi(optarg);
        printf("CAPTURE: %d\r\n", capture_live);
        break;
      case 'l':
        strcpy(capture_path, optarg);
        printf("CAPTURE_PATH: %s\r\n", capture_path);
        break;
      case 'f': //Optional Flag
        strcpy(filter, optarg);
        printf("FILTER: %s\r\n", filter);
        break;
    }
  }

  if (argc <= 1) {
      ROS_INFO("Insufficient args\r\n");
      printUsage(argv[0]);
      return 0;
  }

  if (capture_live == LIVE) {
    printf("RUNNING LIVE\r\n");
  } else if (capture_live == OFFLINE) {
    printf("RUNNING OFFLINE\r\n");
  } else {
    printf("Bad capture option, Definition in parser.h\r\n");
    return 0;
  }
  
  ros::NodeHandle nh;
  initUnfilteredPublisher(nh); 
  
  run(port, packets, interface, filter, capture_live, capture_path);
  ros::spin();
  
  return 0;
}
