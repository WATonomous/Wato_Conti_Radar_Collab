#ifndef PROCESS_PACKET_H
#define PROCESS_PACKET_H

/* File: processPacket.h
 *
 * Groups incoming packets into buffer based on timestamp & type
 * Performs simple filtering & thresholding on simeple packet contents
 */

/* General Use Includes */
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include "parser.h"
#include "radarPublisher.h"

//Return Values
#define SUCCESS             0
#define NO_DETECTIONS       1
#define PUBLISH_FAIL        2
#define CLEAR_FAIL          3
#define BAD_EVENT_ID        4
#define NO_PUBLISHER        5
#define INIT_FAIL           6
#define BAD_PORT            7
#define BAD_SERVICE_ID      8
#define NO_PUBLISHER        9
#define NO_PUB_CLR_FAIL     10
#define FALSE_DETECTION_1   11
#define FALSE_DETECTION_2   12
#define FALSE_DETECTION_3   13
#define TOO_MUCH_NOISE      14
#define SS_DEFECTIVE_HW     15
#define SS_BAD_VOLT         16
#define SS_BAD_TEMP         17
#define SS_GM_MISSING       18
#define SS_PWR_REDUCED      19
#define NO_PROCESS          20

// Logging Verbosity Levels
enum LoggingLevel {LOG_NONE, LOG_DEBUG, LOG_WARNING, LOG_ERROR};

//Scan mode
enum ScanMode {NEAR_FAR_SCAN, NEAR_SCAN, FAR_SCAN}; // 0=Near&Far, 1=NearOnly, 2=FarOnly

//Filter Thresholds
#define SNR_THRESHOLD            3 //dBr
#define VELOCITY_LOWER_THRESHOLD 0.0 // m/s
#define RCS_THRESHOLD            -25 //(dBm)^2
#define DISTANCE_MAX_THRESHOLD   20 //m
#define DISTANCE_MIN_THRESHOLD   0.0 //m
#define AZI_ANGLE_0_THRESHOLD      -1.44862 //radians, 83 degrees Azi Angle 0 is the angle from the left center
#define AZI_ANGLE_1_THRESHOLD      1.44862 //Azi Angle 1 is the angle from the right center, positive according to Conti

class PacketProcessor {
    private:
        class RadarPublisher*    Publisher; //Publisher Object for Callback
        pthread_mutex_t          Mutex; //Mutex for synchronization
        PacketGroup_t            PacketsBuffer[2]; //Double buffer
        uint8_t                  curNearIdx, curFarIdx; //Double buffer indexes
        uint32_t                 curNearTimeStamp, curFarTimeStamp; //Time Stamps
        bool                     publish;
        uint8_t                  scanMode; // 0=Near&Far, 1=NearOnly, 2=FarOnly

        uint8_t clearPackets(uint8_t idx); //Don't expose these
        uint8_t clearAllPackets();
        uint8_t publishPackets(uint8_t idx);

    public:
        PacketProcessor();

        uint8_t initializePacketProcessor(RadarPublisher* newPublisher, uint8_t newscanMode);
        uint8_t processRDIMsg(const radar_driver::RadarPacket::ConstPtr& packet);
        uint8_t processSSPacket(SSPacket_t* packet);
        void    setPublisherCallback(RadarPublisher* newPublisher);
        /* Print the index buffer chosen */
        void    printPacketGroup(uint8_t idx);
};

#endif /* PROCESS_PACKET_H */
