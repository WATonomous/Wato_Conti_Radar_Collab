#ifndef MULTI_RADAR_H
#define MULTI_RADAR_H

/* File: multiRadar.h
 *
 * Drops duplicate points from both radars
 */
#include <ros/ros.h>

//msgs
#include "radar_driver/RadarPacket.h"
#include "radar_driver/RadarDetection.h"

 class MultiRadar {

  public:
    bool right_radar_detection = false;
    bool left_radar_detection = false;
    bool multi_radar_detection = false;
    ros::Timer radar_timer; //Using a timer just because it has a callback

    MultiRadar(ros::NodeHandle &nh); //Constructor
    void detection(const ros::TimerEvent &event); //Main detection function

  private:
    ros::Subscriber right_radar_sub;
    ros::Subscriber left_radar_sub;
    ros::NodeHandle node_;

    radar_driver::RadarPacket right_radar;
    radar_driver::RadarPacket left_radar;

    void rightDetection(const radar_driver::RadarPacket::ConstPtr &right_radar_msg);
    void leftDetection(const radar_driver::RadarPacket::ConstPtr &left_radar_msg);
 };

#endif
