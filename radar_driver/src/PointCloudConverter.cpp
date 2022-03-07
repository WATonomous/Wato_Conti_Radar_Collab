#include "radarPublisher.h"
#include "processPacket.h"
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <boost/algorithm/string.hpp>

//msgs
#include "radar_driver/RadarDetection.h"
#include "radar_driver/RadarPacket.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>

#define BASE_FRAME  "base_link" //the frame of the car
#define MAX_DIST    16 //metres, arbitrary

std::string name = "";
std::string radarFrame = "radar_fixed";
pthread_mutex_t mutex; //Non recursive
tf::Transform fixed;
std::vector<radar_driver::RadarPacket> packets;
ros::Subscriber rawData;
ros::Publisher pcData;
ros::Publisher marker_pub;
visualization_msgs::Marker leftFOVLine, rightFOVline;

uint32_t curTimeStamp = 0;

void packetCloudGenerator(std::vector<radar_driver::RadarPacket> group) {

    if (group.size() == 0) {
        printf("Skipping empty group\r\n");
        return;
    }
    sensor_msgs::PointCloud2 pc;
    pc.header = group[0].header;
    pc.header.frame_id = radarFrame;
    pc.height = 1;
    pc.width = 0;

    //Set x field type
    sensor_msgs::PointField pf;
    pf.name='x';
    pf.offset=0;
    pf.datatype=sensor_msgs::PointField::FLOAT32;
    pf.count=1;
    pc.fields.push_back(pf);

    pf.name='y'; //Same except name & offset, reuse
    pf.offset=4;
    pc.fields.push_back(pf);

    pf.name='z';
    pf.offset=8;
    pc.fields.push_back(pf);

    pf.name="intensity"; //Green = small (low RCS), Red = large (high RCS)
    pf.offset=12;
    pc.fields.push_back(pf);

    pc.is_bigendian = false; //All computer tested on are little endian
    pc.point_step = 16; //4 bytes for x, 4 bytes for y, 4 bytes for z, 4 bytes for intensity
    pc.is_dense = true;

    uint8_t* tmp;
    for (uint8_t i = 0; i < group.size(); i++) {
        for (uint8_t j = 0; j < group[i].Detections.size(); j++) {
            pc.width ++;
            tmp = (uint8_t*)&(group[i].Detections[j].posX); //My computer is little endian (lsb @ lowest index)
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }

            tmp = (uint8_t*)&(group[i].Detections[j].posY);
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }

            tmp = (uint8_t*)&(group[i].Detections[j].posZ);
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }

            //Map from -100/+100 to 0/+100, lower span & more color changes
            float intensity = (group[i].Detections[j].RCS0 / 2.0) + 50.0;
            tmp = (uint8_t*) &(intensity); //Turn float32 into 4 bytes
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }
        }
    }

    pc.row_step = pc.point_step * pc.width;
    leftFOVLine.header.stamp = rightFOVline.header.stamp = pc.header.stamp;

    pcData.publish(pc);

    marker_pub.publish(leftFOVLine);
    marker_pub.publish(rightFOVline);

    return;
}

void radarCallback(const radar_driver::RadarPacket::ConstPtr& msg) {

    if (curTimeStamp == 0) {
        curTimeStamp = msg->TimeStamp;
        packets.push_back(*msg);
    } else if (curTimeStamp != msg->TimeStamp) {
        packetCloudGenerator(packets);
        curTimeStamp = msg->TimeStamp;
        packets.clear();
        packets.push_back(*msg);
    }
    else { // Time Stamps are the same
        packets.push_back(*msg);
    }
    return;
}

void configureFOVlines(void) {
    leftFOVLine.header.frame_id = rightFOVline.header.frame_id = radarFrame;
    //Configure timestamp at each publish
    leftFOVLine.ns = rightFOVline.ns = "points_and_lines";
    leftFOVLine.action = rightFOVline.action = visualization_msgs::Marker::ADD;
    leftFOVLine.pose.orientation.w = rightFOVline.pose.orientation.w = 1.0;

    leftFOVLine.id = 1;
    rightFOVline.id = 2;

    leftFOVLine.type = rightFOVline.type = visualization_msgs::Marker::LINE_STRIP;

    leftFOVLine.scale.x = rightFOVline.scale.x = 0.05;
    leftFOVLine.color.b = rightFOVline.color.b = 1.0;
    leftFOVLine.color.r = rightFOVline.color.r = 1.0;
    leftFOVLine.color.g = rightFOVline.color.g = 1.0;
    leftFOVLine.color.a = rightFOVline.color.a = 0.5;

    geometry_msgs::Point p;
    p.x = p.y = p.z = 0; //Create origin point
    rightFOVline.points.push_back(p);
    leftFOVLine.points.push_back(p);

    p.x = MAX_DIST; //Create farthest point
    p.y = p.x * tan(AZI_ANGLE_1_THRESHOLD);
    rightFOVline.points.push_back(p);
    p.y *= -1;
    leftFOVLine.points.push_back(p);
}

int main(int argc, char **argv)
{
    std::string packetTopic = "";
    int c = 0;

    // Get the command line option, if any
    while ((c = getopt (argc, argv, "+hn:t:")) != -1) { //Pass remainder of arguments to the sniffer
        switch (c) {
            case 'h':
                printf("\nUsage: %s [-h] [-t Topic Name]\r\n", argv[0]);
                printf("\tt: Input radar Topic\r\n");
                exit(0);
                break;
            case 't':
                packetTopic = std::string(optarg);
                printf("Converting Data From Topic: %s\r\n", packetTopic);
                break;
            case 'n':
                name = std::string(optarg);
                printf("Using namespace:: %s\r\n", name);
                break;
        }
    }

    ros::init(argc, argv, "radar_visualizer");

    ros::NodeHandle n;

    int x = 0;
    int y = 0;
    int z = 0;
    if (boost::iequals(name,"radar_left")) {
        radarFrame = "radar_left_frame";
    }
    else if (boost::iequals(name,"radar_right")) {
        radarFrame = "radar_right_frame";
    }

    mutex = PTHREAD_MUTEX_INITIALIZER; //Non recursive
    fixed.setOrigin( tf::Vector3(x, y, z) ); //Set transform to be 0 rot, 0 trans
    fixed.setRotation( tf::Quaternion(0, 0, 0, 1) );

    pcData   = n.advertise<sensor_msgs::PointCloud2>(std::string("radar_pointcloud"), 100);
    rawData  = n.subscribe(packetTopic, 100, radarCallback);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    packets.clear();

    configureFOVlines();

    ros::spin();

    return 0;
}
