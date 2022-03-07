/* General Use Includes */
#include "processPacket.h"
#include <cstring>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>

//Processing Rules: Written out in better detail in the spec doc
//Spec Doc: https://docs.google.com/document/d/13W7H_Z5pgCx8rVgnv7u1MaKYaGQvnPspWVuuTmzJgUA/edit

//#define PROCESS_SS_PACKET //Perception doesn't need it

static PacketGroup_t EmptyPackets; //Local static only, Cleanup struct

//Some local functions
bool loadRDIMessageFromPacket(radar_driver::RadarPacket* newMsg, const radar_driver::RadarPacket::ConstPtr& oldMsg);
void loadSSMessage(radar_driver::SensorStatus* msg, SSPacket_t* packet);

PacketProcessor::PacketProcessor() {
    Mutex = PTHREAD_MUTEX_INITIALIZER; //Non recursive
    PacketsBuffer[2]; //Init Double Buffer
    Publisher = NULL; //Publisher object for callback
    publish = false; //Publishing flag
    curNearTimeStamp = 0, curFarTimeStamp = 0;
    curNearIdx = 0, curFarIdx = 0;
    scanMode = NEAR_FAR_SCAN; // 0=Near&Far, 1=NearOnly, 2=FarOnly
}

/* Initialization Processor
*
*/
uint8_t PacketProcessor::initializePacketProcessor(RadarPublisher* newPublisher, uint8_t newscanMode) {
    pthread_mutex_lock(&Mutex);

    if (clearAllPackets()) { //Wipe internal struct
        pthread_mutex_unlock(&Mutex);
        return INIT_FAIL;
    }

    scanMode = newscanMode;
    Publisher = newPublisher;
    //All packet structs should be 0'd by default
    pthread_mutex_unlock(&Mutex);

    return SUCCESS;
}

uint8_t PacketProcessor::processRDIMsg(const radar_driver::RadarPacket::ConstPtr& packet) {
    if ( packet->Detections.size() < 1 ){
        return NO_DETECTIONS;
    }

    //Only mutex here b/c not using the Packets struct yet
    pthread_mutex_lock(&Mutex);

    if(packet->EventID == FAR1 || packet->EventID == FAR0) {
        if (scanMode != NEAR_SCAN) {
            if (curFarTimeStamp == 0){ //Init Case
                curFarTimeStamp = packet->TimeStamp;
            } else if (curFarTimeStamp != packet->TimeStamp) { //New timestamp
                curFarIdx = (curFarIdx + 1) % 2;

                if (scanMode == FAR_SCAN || curFarIdx == curNearIdx) { //Indexes are the same
                    // We just got a new TS for both near & far, so we should publish the old buffer
                    publish = true;
                }
            }

            PacketGroup_t * curGroup = &PacketsBuffer[curFarIdx]; //Tmp to make code easier to read
            if (loadRDIMessageFromPacket(&curGroup->farPackets[curGroup->numFarPackets], packet)) {
                curGroup->numFarPackets++;
            }

            if (publish) {
                uint8_t err = publishPackets((curFarIdx+1)%2); //Publish the previous buffer idx
                publish = false;
                pthread_mutex_unlock(&Mutex);
                return err;
            }
        }
    } else if (packet->EventID == NEAR0 || packet->EventID == NEAR1 || packet->EventID == NEAR2) {
        if (scanMode != FAR_SCAN) {
            if (curNearTimeStamp == 0){ //Init Case
                curNearTimeStamp = packet->TimeStamp;
            } else if (curNearTimeStamp != packet->TimeStamp) { //New timestamp
                curNearIdx = (curNearIdx + 1) % 2;

                if (scanMode == NEAR_SCAN || curNearIdx == curFarIdx) { //Indexes are the same
                    // We just got a new TS for both near & far, so we should publish the old buffer
                    publish = true;
                }
            }

            PacketGroup_t* curGroup = &PacketsBuffer[curNearIdx]; //Tmp to make code easier to read
            if (loadRDIMessageFromPacket(&curGroup->nearPackets[curGroup->numNearPackets], packet)) {
                curGroup->numNearPackets++;
            }

            if (publish) {
                uint8_t err = publishPackets((curNearIdx+1)%2); //Publish the previous buffer idx
                publish = false;
                pthread_mutex_unlock(&Mutex);
                return err;
            }
        }
    } else {
        pthread_mutex_unlock(&Mutex);
        return BAD_EVENT_ID;
    }

    pthread_mutex_unlock(&Mutex);
    return SUCCESS;
}

/* Process Sensor Status Packet
* -- Add packet to our internal struct
* -- Call our publisher to publish all packets since we have a group
* -- Publish as soon as we have the sensor status packet
*/
uint8_t PacketProcessor::processSSPacket(SSPacket_t* packet) {
    // TODO: Raise an alarm on the dashboard with error info.
    if (packet->Defective && DEFECTIVE_HW) {
        return SS_DEFECTIVE_HW;
    } else if (packet->BadSupplyVolt && BAD_VOLTAGE) {
        return SS_BAD_VOLT;
    } else if (packet->BadTemp && BAD_TEMP) {
        return SS_BAD_TEMP;
    } else if (packet->GmMissing && GM_MISSING) {
        return SS_GM_MISSING;
    } else if (packet->TxPowerStatus && POWER_REDUCED) {
        return SS_PWR_REDUCED;
    }
    pthread_mutex_lock(&Mutex);

    //TODO: Implement Error Checking Here...

#if PROCESS_SS_PACKET
    loadSSMessage(&Packets.sensorStatusMsg, packet);
#endif

    pthread_mutex_unlock(&Mutex);
    return SUCCESS;
}

/* Publish Packets
* -- Send our packet struct to the ROS publisher
* -- Clear our intial struct & reset
* -- Only called ffrom synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::publishPackets(uint8_t idx) {

    if (Publisher == NULL) { //Publisher not set up
        //Check if we can still clear the packets
        if (clearPackets(idx)) { //Everything failed
            return NO_PUB_CLR_FAIL;
        }
        return NO_PUBLISHER; //Only null publisher, cleared ok
    } else if (Publisher->pubCallback(&PacketsBuffer[idx])) {
        return PUBLISH_FAIL;
    }

    if (clearPackets(idx)) {
        return CLEAR_FAIL;
    }
    return SUCCESS;
}

/* Clear Packet Struct
* -- Empty one internal packet struct
* -- Only called form synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::clearPackets(uint8_t idx) {
    if (idx < 2) {
        PacketsBuffer[idx] = EmptyPackets; //Replace w/ empty struct
        PacketsBuffer[idx].numFarPackets = 0;
        PacketsBuffer[idx].numNearPackets = 0;
        return SUCCESS;
    }
    return CLEAR_FAIL;
}

/* Clear All Packet Buffers
* -- Empty both packet structs
* -- Only called form synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::clearAllPackets() {
    clearPackets(0);
    clearPackets(1);

    return SUCCESS;
}

bool loadRDIMessageFromPacket(radar_driver::RadarPacket* newMsg, const radar_driver::RadarPacket::ConstPtr& oldMsg) {
    newMsg->EventID                    = oldMsg->EventID;
    newMsg->TimeStamp                  = oldMsg->TimeStamp;
    newMsg->MeasurementCounter         = oldMsg->MeasurementCounter;
    newMsg->Vambig                     = oldMsg->Vambig;
    newMsg->CenterFrequency            = oldMsg->CenterFrequency;
    newMsg->Detections.clear();

    for(uint8_t i = 0; i < oldMsg->Detections.size(); i++) {

        // TODO: Figure out an SNR threshold value that actually works here.
        if (oldMsg->Detections[i].SNR < SNR_THRESHOLD) { // Too much noise; drop detection.
            continue;
        } else if (abs(oldMsg->Detections[i].VrelRad) < VELOCITY_LOWER_THRESHOLD) {
            continue;
        } else if (oldMsg->Detections[i].posX > DISTANCE_MAX_THRESHOLD) { // need to do trig
            continue;
        } else if (oldMsg->Detections[i].posX < DISTANCE_MIN_THRESHOLD) {
            continue;
        } else if (oldMsg->Detections[i].RCS0 > RCS_THRESHOLD) {
            continue;
        } else if (oldMsg->EventID == FAR1 || oldMsg->EventID == FAR0) {
            //limit far scan to 9 degrees, according to conti
            if (oldMsg->Detections[i].AzAng0 < -0.15708 || oldMsg->Detections[i].AzAng1 > 0.15708) {
                continue;
            }
        } else if(oldMsg->Detections[i].AzAng0 < AZI_ANGLE_0_THRESHOLD || oldMsg->Detections[i].AzAng1 > AZI_ANGLE_1_THRESHOLD){
            continue;
        }

        radar_driver::RadarDetection data;

        // Copy all data from old message detection to new message

        data.Pdh0          = oldMsg->Detections[i].Pdh0;

        data.AzAng0        = oldMsg->Detections[i].AzAng0;
        data.RCS0          = oldMsg->Detections[i].RCS0;
        data.AzAngVar0     = oldMsg->Detections[i].AzAngVar0;
        data.Prob0         = oldMsg->Detections[i].Prob0;

        data.AzAng1        = oldMsg->Detections[i].AzAng1;
        data.RCS1          = oldMsg->Detections[i].RCS1;
        data.AzAngVar1     = oldMsg->Detections[i].AzAngVar1;
        data.Prob1         = oldMsg->Detections[i].Prob1;

        data.VrelRad      = oldMsg->Detections[i].VrelRad;
        data.ElAng        = oldMsg->Detections[i].ElAng;
        data.RangeVar     = oldMsg->Detections[i].RangeVar;
        data.VrelRadVar   = oldMsg->Detections[i].VrelRadVar;
        data.ElAngVar     = oldMsg->Detections[i].ElAngVar;
        data.SNR          = oldMsg->Detections[i].SNR;
        data.Range        = oldMsg->Detections[i].Range;

        data.posX = oldMsg->Detections[i].posX;
        data.posY = oldMsg->Detections[i].posY;
        data.posZ = oldMsg->Detections[i].posZ;

        newMsg->Detections.push_back(data);
    }

    // Return true if there is at least one detection in newMsg after filtering
    return (newMsg->Detections.size() > 0);
}

/* Load SS ROS Message
* -- Local Function, not in class definition
*/
void loadSSMessage(radar_driver::SensorStatus* msg, SSPacket_t* packet) {
    msg->PartNumber             = packet->PartNumber;
    msg->AssemblyPartNumber     = packet->AssemblyPartNumber;
    msg->SWPartNumber           = packet->SWPartNumber;
    for (uint8_t i = 0; i < SENSOR_SERIAL_NUM_LEN; i++) {
        msg->SerialNumber[i]    = packet->SerialNumber[i]; //Should work for a boost::array object
    }
    msg->BLVersion              = packet->BLVersion;
    msg->SWVersion              = packet->SWVersion;
    msg->UTCTimeStamp           = packet->UTCTimeStamp;
    msg->TimeStamp              = packet->TimeStamp;
    msg->SurfaceDamping         = packet->SurfaceDamping;
    msg->OpState                = packet->OpState;
    msg->CurrentFarCF           = packet->CurrentFarCF;
    msg->CurrentNearCF          = packet->CurrentNearCF;
    msg->Defective              = packet->Defective;
    msg->BadSupplyVolt          = packet->BadSupplyVolt;
    msg->BadTemp                = packet->BadTemp;
    msg->GmMissing              = packet->GmMissing;
    msg->TxPowerStatus          = packet->TxPowerStatus;
    msg->MaximumRangeFar        = packet->MaxRangeFar;
    msg->MaximumRangeNear       = packet->MaxRangeNear;
}
/* Set ROS publisher callback
*/
void PacketProcessor::setPublisherCallback(RadarPublisher* newPublisher) {
    pthread_mutex_lock(&Mutex);
    Publisher = newPublisher;
    pthread_mutex_unlock(&Mutex);
}

/* Print the currently selected buffer
*/
void PacketProcessor::printPacketGroup(uint8_t idx) {
    pthread_mutex_lock(&Mutex);
    if (idx >= 2) {
        printf("Improper Idx Given: %u\r\n", idx);
        return;
    }
    PacketGroup_t * Packets = &PacketsBuffer[idx];

    for (uint8_t j = 0; j < Packets->numFarPackets; j++) {
        printf("\nPROC:Far Packet: %d, len: %u\n", j, Packets->farPackets[j].Detections.size());
        for(uint8_t i = 0; i < Packets->farPackets[j].Detections.size(); i++) {
            printf("PROC:RDI Idx: %d \n"    , i);
            printf("PROC:posX %f \n"        , Packets->farPackets[j].Detections[i].posX);
            printf("PROC:posY %f \n"        , Packets->farPackets[j].Detections[i].posY);
            printf("PROC:posZ %f \n"        , Packets->farPackets[j].Detections[i].posZ);
            printf("PROC:VrelRad %f \n"     , Packets->farPackets[j].Detections[i].VrelRad);
            printf("PROC:AzAng0 %f \n"       , Packets->farPackets[j].Detections[i].AzAng0);
            printf("PROC:AzAng1 %f \n"       , Packets->farPackets[j].Detections[i].AzAng1);
            printf("PROC:ElAng %f \n"       , Packets->farPackets[j].Detections[i].ElAng);
            printf("PROC:RCS0 %f \n"         , Packets->farPackets[j].Detections[i].RCS0);
            printf("PROC:RCS1 %f \n"         , Packets->farPackets[j].Detections[i].RCS1);
            printf("PROC:RangeVar %f \n"    , Packets->farPackets[j].Detections[i].RangeVar);
            printf("PROC:VrelRadVar %f \n"  , Packets->farPackets[j].Detections[i].VrelRadVar);
            printf("PROC:AzAngVar0 %f \n"    , Packets->farPackets[j].Detections[i].AzAngVar0);
            printf("PROC:AzAngVar1 %f \n"    , Packets->farPackets[j].Detections[i].AzAngVar1);
            printf("PROC:ElAngVar %f \n"    , Packets->farPackets[j].Detections[i].ElAngVar);
            printf("PROC:Prob0 %f \n"       , Packets->farPackets[j].Detections[i].Prob0);
            printf("PROC:Prob1 %f \n"       , Packets->farPackets[j].Detections[i].Prob1);
            printf("PROC:SNR %f \n"         , Packets->farPackets[j].Detections[i].SNR);
            printf("\n");
        }
    }

    for (uint8_t j = 0; j < Packets->numNearPackets; j++) {
        printf("PROC:Near Packet: %d, len: %u\n", j, Packets->nearPackets[j].Detections.size());
        for(uint8_t i = 0; i < Packets->nearPackets[j].Detections.size(); i++) {
            printf("PROC:RDI Idx: %d \n"    , i);
            printf("PROC:posX %f \n"        , Packets->nearPackets[j].Detections[i].posX);
            printf("PROC:posY %f \n"        , Packets->nearPackets[j].Detections[i].posY);
            printf("PROC:posZ %f \n"        , Packets->nearPackets[j].Detections[i].posZ);
            printf("PROC:VrelRad %f \n"     , Packets->nearPackets[j].Detections[i].VrelRad);
            printf("PROC:AzAng0 %f \n"       , Packets->nearPackets[j].Detections[i].AzAng0);
            printf("PROC:AzAng1 %f \n"       , Packets->nearPackets[j].Detections[i].AzAng1);
            printf("PROC:ElAng %f \n"       , Packets->nearPackets[j].Detections[i].ElAng);
            printf("PROC:RCS0 %f \n"         , Packets->nearPackets[j].Detections[i].RCS0);
            printf("PROC:RCS1 %f \n"         , Packets->nearPackets[j].Detections[i].RCS1);
            printf("PROC:RangeVar %f \n"    , Packets->nearPackets[j].Detections[i].RangeVar);
            printf("PROC:VrelRadVar %f \n"  , Packets->nearPackets[j].Detections[i].VrelRadVar);
            printf("PROC:AzAngVar0 %f \n"    , Packets->nearPackets[j].Detections[i].AzAngVar0);
            printf("PROC:AzAngVar1 %f \n"    , Packets->nearPackets[j].Detections[i].AzAngVar1);
            printf("PROC:ElAngVar %f \n"    , Packets->nearPackets[j].Detections[i].ElAngVar);
            printf("PROC:Prob0 %f \n"       , Packets->nearPackets[j].Detections[i].Prob0);
            printf("PROC:Prob1 %f \n"       , Packets->nearPackets[j].Detections[i].Prob1);
            printf("PROC:SNR %f \n"         , Packets->nearPackets[j].Detections[i].SNR);
            printf("\n");
        }
    }

    pthread_mutex_lock(&Mutex);
}
