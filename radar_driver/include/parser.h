#ifndef PARSER_H
#define PARSER_H

/* File: parser.h
 *
 * Parses raw char string packet data into predefined structs
 * Defines metadata & operational parameters for whole pipeline
 */

/* General Use Includes */
#include <stdlib.h>
#include <stdint.h>
#include "sniffer.h"
#include "ros/ros.h"
#include "ros/console.h"

/* Libpcap includes */
#include <pcap.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/ip_icmp.h>

/* ROS Messages */
#include "radar_driver/RadarDetection.h"
#include "radar_driver/RadarPacket.h"
#include "radar_driver/SensorStatus.h"

//Packet Cap Definitions: If we are using live capture or from pcap doc
#define OFFLINE         0
#define LIVE            1

//Service ID Definitions
#define RDI_PACKET_ID  220
#define SS_PACKET_ID   200

// Event ID Definitions
#define FAR0    1
#define FAR1    2
#define NEAR0   3
#define NEAR1   4
#define NEAR2   5

//General Definitions
#define SENSOR_SERIAL_NUM_LEN   26
#define RDI_ARRAY_LEN           38
#define RDI_ARRAY_LEN_NEAR_2    32

//Size Defs
#define NUM_FAR         12
#define NUM_NEAR        18

//Pdh0 Probability flag masks
//Usage: bool x = pdh0 & NEAR_PROB_MASK;
#define SIDELOBE_PROB_MASK   0x04
#define INFERENCE_PROB_MASK  0x02
#define NEAR_PROB_MASK       0x01

// flag masks for SS Packet fields.
#define DEFECTIVE_HW        0x01
#define BAD_VOLTAGE         0x01
#define BAD_TEMP            0x01
#define GM_MISSING          0x01
#define POWER_REDUCED       0x01

//How do we want to use these for integration?
//Kind of an arbitrary choice to use enums here
enum RadarTxPorts { //Enum b/c numerous related values, cleaner than defines
    RADAR_1_PORT = 31120,     RADAR_2_PORT = 31121,    RADAR_3_PORT = 31122,
    //31123 Reserved for Rx port (since its the default)
    RADAR_4_PORT = 31124,     RADAR_5_PORT = 31125,    RADAR_6_PORT = 31126,
};

//General Struct Defs for the Radar UDP Packets
typedef struct udphdr udphdr_t; //udp.h struct, typedef for clarity

//SOME/IP Header
typedef struct packetHeader
{
    uint8_t     service_ID;
    uint8_t     event_ID;
    uint32_t    length;
    //request ID is split into client ID (16)and subscriber ID(16)
    uint32_t    request_ID;
    uint8_t     protocol_ver;
    uint8_t     interface_ver;
    uint8_t     message_Type;
    uint8_t     return_Code;
} packetHeader_t;

//Radar Packet Header
typedef struct payloadHeader
{
    uint8_t     EventID;
    uint16_t    CRC;
    uint16_t    Len;
    uint8_t     SQC;
    uint8_t     MessageCounter;
    uint64_t    UTCTimeStamp;
    uint32_t    TimeStamp;
    uint32_t    MeasurementCounter;
    uint32_t    CycleCounter;
    uint16_t    NofDetections;
    int16_t     VambigRaw;
    uint8_t     CenterFrequencyRaw;
    RadarTxPorts RadarPortNum;
    uint8_t     DetectionsInPacket;
    float       Vambig;
    float       CenterFrequency;
} payloadHeader_t;

//Int RDI Data
typedef struct packetRDIRawData {
    uint16_t    f_Range;
    int16_t     f_VrelRad;
    int16_t     f_AzAng0;
    int16_t     f_AzAng1;
    int16_t     f_ElAng;
    int16_t     f_RCS0;
    int16_t     f_RCS1;
    uint8_t     f_Prob0;
    uint8_t     f_Prob1;
    uint16_t    f_RangeVar;
    uint16_t    f_VrelRadVar;
    uint16_t    f_AzAngVar0;
    uint16_t    f_AzAngVar1;
    uint16_t    f_ElAngVar;
    uint8_t     ui_Pdh0;
    uint8_t     f_SNR;
} packetRDIRawData_t;

//float RDI Data
typedef struct packetRDIData {
    float f_Range;
    float f_VrelRad;
    float f_AzAng0;
    float f_AzAng1;
    float f_ElAng;
    float f_RCS0;
    float f_RCS1;
    float f_Prob0;
    float f_Prob1;
    float f_RangeVar;
    float f_VrelRadVar;
    float f_AzAngVar0;
    float f_AzAngVar1;
    float f_ElAngVar;
    uint8_t ui_Pdh0;
    float f_SNR;
    float X; //Fill these in processor
    float Y;
    float Z;
} packetRDIData_t;

typedef struct RDIPacket{
    payloadHeader_t payloadHeaderData;
    packetRDIRawData_t listDataArray[RDI_ARRAY_LEN];
    packetRDIData_t resListDataArray[RDI_ARRAY_LEN];
} RDIPacket_t;

//Sensor Status Data, slightly different structure than RDI packet
typedef struct SSPacket {
    uint16_t    CRC;
    uint16_t    len;
    uint8_t     SQC;
    uint64_t    PartNumber;
    uint64_t    AssemblyPartNumber;
    uint64_t    SWPartNumber;
    uint8_t     SerialNumber[SENSOR_SERIAL_NUM_LEN];
    uint32_t    BLVersion; //24 Bits unsigned in spreadsheet
    uint32_t    BL_CRC;
    uint32_t    SWVersion; //24 Bits unsigned in spreadsheet
    uint32_t    SW_CRC;
    uint64_t    UTCTimeStamp;
    uint32_t    TimeStamp;
    uint32_t    SurfaceDamping_Raw;
    uint8_t     OpState;
    uint8_t     CurrentFarCF;
    uint8_t     CurrentNearCF;
    uint8_t     Defective;
    uint8_t     BadSupplyVolt;
    uint8_t     BadTemp;
    uint8_t     GmMissing;
    uint8_t     TxPowerStatus;
    uint16_t    MaxRangeFar_Raw;
    uint16_t    MaxRangeNear_Raw;
    float      SurfaceDamping; //Combining res data w/ int b/c not alot of res data
    float      MaxRangeFar;
    float      MaxRangeNear;
} SSPacket_t;

//Static # of near or far packets, dynamic # of detections per packet
typedef struct PacketGroup {
    radar_driver::RadarPacket   nearPackets[NUM_NEAR];
    radar_driver::RadarPacket   farPackets[NUM_FAR];
    uint8_t numFarPackets;
    uint8_t numNearPackets;
} PacketGroup_t;

uint8_t parse_packet(udphdr_t* udphdr, unsigned char* packetptr); //Parser in parser.cpp
void    initUnfilteredPublisher(ros::NodeHandle nh_new);
void    publishRDIPacket(RDIPacket_t * packet);
void    loadPacketMsg(RDIPacket_t * packet, radar_driver::RadarPacket * msg);
void    loadSSMessage(SSPacket_t* packet, radar_driver::SensorStatus* msg);

#endif /* PARSER_H */
