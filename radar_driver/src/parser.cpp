/* General Use Includes */
#include "resMultipliers.h"
#include "processPacket.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <inttypes.h>
#include <ros/ros.h>
#include "radar_driver/RadarPacket.h"
#include "radar_driver/SensorStatus.h"

#define DETECTIONS_IN_ELEMENT_BYTE_POS 55
#define PARSE_SS

//Resolution Multipliers, make static const global since never changing
static const resMultipliersRDI_t resRDIMults;
static const resMultipliersSS_t  resSSMults;

//Unfiltered ROS publisher
static ros::Publisher unfilteredPublisher;

void initUnfilteredPublisher(ros::NodeHandle nh) {
    unfilteredPublisher  = nh.advertise<radar_driver::RadarPacket>("unfiltered_radar_packet", 50);
}

uint8_t parse_packet(udphdr_t* udphdr, unsigned char* packetptr) {

    packetHeader_t Header; //Create the header locally

    //128 bit (16 Byte) Packet Header
    //Were not really using this, decoding for completion's sake
    Header.service_ID      = (packetptr[8] << 8) | (packetptr[9]); //Big Endian
    Header.event_ID        = (packetptr[10] << 8) | packetptr[11]; // Indicates far or near scan
    Header.length          = (packetptr[12] << 24) | (packetptr[13] << 16) | (packetptr[14] << 8) | packetptr[15]; //8 bytes past this
    Header.request_ID      = (packetptr[16] << 24) | (packetptr[17] << 16) | (packetptr[18] << 8) | packetptr[19];
    Header.protocol_ver    =  packetptr[20];
    Header.interface_ver   =  packetptr[21];
    Header.message_Type    =  packetptr[22];
    Header.return_Code     =  packetptr[23];

    if (Header.service_ID == RDI_PACKET_ID) {
        if (packetptr[DETECTIONS_IN_ELEMENT_BYTE_POS] == 0) {
            return NO_DETECTIONS;
        }

        RDIPacket_t RDI_Packet; //Create RDI Packet struct

        //256 Bit (32 Byte) Payload Header
        RDI_Packet.payloadHeaderData.EventID            = Header.event_ID; //Duplicated b/c useful later
        RDI_Packet.payloadHeaderData.CRC                = (packetptr[24] << 8) | (packetptr[25]);     
        RDI_Packet.payloadHeaderData.Len                = (packetptr[26] << 8) | (packetptr[27]);           
        RDI_Packet.payloadHeaderData.SQC                =  packetptr[28];
        RDI_Packet.payloadHeaderData.MessageCounter     =  packetptr[29];
        RDI_Packet.payloadHeaderData.UTCTimeStamp       = (packetptr[30] << 56) | (packetptr[31] << 48) | (packetptr[32] << 40) | (packetptr[33] << 32) | (packetptr[34] << 24) | (packetptr[35] << 16) | (packetptr[36] << 8) | packetptr[37];
        RDI_Packet.payloadHeaderData.TimeStamp          = (packetptr[38] << 24) | (packetptr[39] << 16) | (packetptr[40] << 8) | packetptr[41];
        RDI_Packet.payloadHeaderData.MeasurementCounter = (packetptr[42] << 24) | (packetptr[43] << 16) | (packetptr[44] << 8) | packetptr[45];
        RDI_Packet.payloadHeaderData.CycleCounter       = (packetptr[46] << 24) | (packetptr[47] << 16) | (packetptr[48] << 8) | packetptr[49];
        RDI_Packet.payloadHeaderData.NofDetections      = (packetptr[50] << 8) | packetptr[51];
        RDI_Packet.payloadHeaderData.VambigRaw          = (packetptr[52] << 8) | packetptr[53];
        RDI_Packet.payloadHeaderData.CenterFrequencyRaw =  packetptr[54];
        RDI_Packet.payloadHeaderData.DetectionsInPacket =  packetptr[55];
        RDI_Packet.payloadHeaderData.Vambig             = RDI_Packet.payloadHeaderData.VambigRaw            * resRDIMults.VambigRes;
        RDI_Packet.payloadHeaderData.CenterFrequency    = RDI_Packet.payloadHeaderData.CenterFrequencyRaw   * resRDIMults.CenterFreqRes;

        //7168 - 8512 Bit (896 - 1064 Byte) Radar Dection Payload. Size depends on eventID. 
        int listStartIndex = 56;
        for (int i = 0; i < RDI_Packet.payloadHeaderData.DetectionsInPacket; i++) { //Only fill as many as we have valid packets, dont bother filling 0's

            //Radar Detection Images are 224 Bits (28 Bytes) each
            RDI_Packet.listDataArray[i].f_Range      = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_VrelRad    = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_AzAng0     = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_AzAng1     = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_ElAng      = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_RCS0       = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_RCS1       = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_Prob0      =  packetptr[listStartIndex++];
            RDI_Packet.listDataArray[i].f_Prob1      =  packetptr[listStartIndex++];
            RDI_Packet.listDataArray[i].f_RangeVar   = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_VrelRadVar = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_AzAngVar0  = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_AzAngVar1  = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].f_ElAngVar   = (packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++]);
            RDI_Packet.listDataArray[i].ui_Pdh0      =  packetptr[listStartIndex++];
            RDI_Packet.listDataArray[i].f_SNR        =  packetptr[listStartIndex++];


            //Add in the proper resolution
            RDI_Packet.resListDataArray[i].f_Range      = RDI_Packet.listDataArray[i].f_Range      * resRDIMults.f_RangeRes;
            RDI_Packet.resListDataArray[i].f_VrelRad    = RDI_Packet.listDataArray[i].f_VrelRad    * resRDIMults.f_VrelRadRes;
            RDI_Packet.resListDataArray[i].f_AzAng0     = RDI_Packet.listDataArray[i].f_AzAng0     * resRDIMults.f_AzAng0Res;
            RDI_Packet.resListDataArray[i].f_AzAng1     = RDI_Packet.listDataArray[i].f_AzAng1     * resRDIMults.f_AzAng1Res;
            RDI_Packet.resListDataArray[i].f_ElAng      = RDI_Packet.listDataArray[i].f_ElAng      * resRDIMults.f_ElAngRes;
            RDI_Packet.resListDataArray[i].f_RCS0       = RDI_Packet.listDataArray[i].f_RCS0       * resRDIMults.f_RCS0Res;
            RDI_Packet.resListDataArray[i].f_RCS1       = RDI_Packet.listDataArray[i].f_RCS1       * resRDIMults.f_RCS1Res;
            RDI_Packet.resListDataArray[i].f_Prob0      = RDI_Packet.listDataArray[i].f_Prob0      * resRDIMults.f_Prob0Res;
            RDI_Packet.resListDataArray[i].f_Prob1      = RDI_Packet.listDataArray[i].f_Prob1      * resRDIMults.f_Prob1Res;
            RDI_Packet.resListDataArray[i].f_RangeVar   = RDI_Packet.listDataArray[i].f_RangeVar   * resRDIMults.f_RangeVarRes;
            RDI_Packet.resListDataArray[i].f_VrelRadVar = RDI_Packet.listDataArray[i].f_VrelRadVar * resRDIMults.f_VrelRadVarRes;
            RDI_Packet.resListDataArray[i].f_AzAngVar0  = RDI_Packet.listDataArray[i].f_AzAngVar0  * resRDIMults.f_AzAngVar0Res;
            RDI_Packet.resListDataArray[i].f_AzAngVar1  = RDI_Packet.listDataArray[i].f_AzAngVar1  * resRDIMults.f_AzAngVar1Res;
            RDI_Packet.resListDataArray[i].f_ElAngVar   = RDI_Packet.listDataArray[i].f_ElAngVar   * resRDIMults.f_ElAngVarRes;
            RDI_Packet.resListDataArray[i].ui_Pdh0      = RDI_Packet.listDataArray[i].ui_Pdh0;
            RDI_Packet.resListDataArray[i].f_SNR        = RDI_Packet.listDataArray[i].f_SNR        * resRDIMults.f_SNRRes;
        }
    
        //Send packet to get published
        publishRDIPacket(&RDI_Packet);

    } else if (Header.service_ID == SS_PACKET_ID) {
#ifdef PARSE_SS
        SSPacket_t SS_Packet;

        SS_Packet.CRC                 = (packetptr[24] << 8) | (packetptr[25]);
        SS_Packet.len                 = (packetptr[26] << 8) | (packetptr[27]);
        SS_Packet.SQC                 = packetptr[28];
        SS_Packet.PartNumber          = (packetptr[29] << 56) | (packetptr[30] << 48) | (packetptr[31] << 40) | (packetptr[32] << 32) | (packetptr[33] << 24) | (packetptr[34] << 16) | (packetptr[35] << 8) | packetptr[36];
        SS_Packet.AssemblyPartNumber  = (packetptr[37] << 56) | (packetptr[38] << 48) | (packetptr[39] << 40) | (packetptr[40] << 32) | (packetptr[41] << 24) | (packetptr[42] << 16) | (packetptr[43] << 8) | packetptr[44];
        SS_Packet.SWPartNumber        = (packetptr[45] << 56) | (packetptr[46] << 48) | (packetptr[47] << 40) | (packetptr[48] << 32) | (packetptr[49] << 24) | (packetptr[50] << 16) | (packetptr[51] << 8) | packetptr[52];
        
        for (uint8_t i = 0; i < SENSOR_SERIAL_NUM_LEN; i++) {//26 Byte Array of uint8_t's
            SS_Packet.SerialNumber[i]   = packetptr[53+i];
        }
        
        SS_Packet.BLVersion           = (packetptr[79] << 16) | (packetptr[80] << 8) | packetptr[81];
        SS_Packet.BL_CRC              = (packetptr[82] << 24) | (packetptr[83] << 16) | (packetptr[84] << 8) | packetptr[85];
        SS_Packet.SWVersion           = (packetptr[86] << 16) | (packetptr[87] << 8) | packetptr[88];
        SS_Packet.SW_CRC              = (packetptr[89] << 24) | (packetptr[90] << 16) | (packetptr[91] << 8) | packetptr[92];
        SS_Packet.UTCTimeStamp        = (packetptr[93] << 56) | (packetptr[94] << 48) | (packetptr[95] << 40) | (packetptr[96] << 32) | (packetptr[97] << 24) | (packetptr[98] << 16) | (packetptr[99] << 8) | packetptr[100];
        SS_Packet.TimeStamp           = (packetptr[101] << 24) | (packetptr[102] << 16) | (packetptr[103] << 8) | packetptr[104];
        SS_Packet.SurfaceDamping_Raw  = (packetptr[105] << 24) | (packetptr[106] << 16) | (packetptr[107] << 8) | packetptr[108];
        SS_Packet.OpState             = packetptr[109];
        SS_Packet.CurrentFarCF        = packetptr[110];
        SS_Packet.CurrentNearCF       = packetptr[111];
        SS_Packet.Defective           = packetptr[112];
        SS_Packet.BadSupplyVolt       = packetptr[113];
        SS_Packet.BadTemp             = packetptr[114];
        SS_Packet.GmMissing           = packetptr[115];
        SS_Packet.TxPowerStatus       = packetptr[116];
        SS_Packet.MaxRangeFar_Raw     = (packetptr[117] << 8) | (packetptr[118]);
        SS_Packet.MaxRangeNear_Raw    = (packetptr[119] << 8) | (packetptr[120]);

        //Apply resolution to relevant members
        SS_Packet.SurfaceDamping      = SS_Packet.SurfaceDamping_Raw * resSSMults.surfaceDampingRes;
        SS_Packet.MaxRangeNear        = SS_Packet.MaxRangeNear_Raw   * resSSMults.MaxRangeNearRes;
        SS_Packet.MaxRangeFar         = SS_Packet.MaxRangeFar_Raw    * resSSMults.MaxRangeFarRes;

        //Don't bother publishing yet since Perception doesn't need it
        //TODO: Figure out who needs these & build the publishing code
#endif
    } else {
        return BAD_SERVICE_ID;
    }

    return SUCCESS;
}

void publishRDIPacket(RDIPacket_t * packet) {
    radar_driver::RadarPacket msg;
    loadPacketMsg(packet, &msg);
    printf("Publishing new message: %d\n", packet->payloadHeaderData.DetectionsInPacket);
    unfilteredPublisher.publish(msg);
}

void loadPacketMsg(RDIPacket_t * packet, radar_driver::RadarPacket * msg) {
    msg->EventID                    = packet->payloadHeaderData.EventID;
    msg->TimeStamp                  = packet->payloadHeaderData.TimeStamp;
    msg->MeasurementCounter         = packet->payloadHeaderData.MeasurementCounter;
    msg->Vambig                     = packet->payloadHeaderData.Vambig;
    msg->CenterFrequency            = packet->payloadHeaderData.CenterFrequency;

    for(uint8_t i = 0; i < packet->payloadHeaderData.DetectionsInPacket; i++) {
        radar_driver::RadarDetection data;

        //Filter through error flags in packet
        if (packet->resListDataArray[i].ui_Pdh0 && NEAR_PROB_MASK) {
            continue;
        } else if (packet->resListDataArray[i].ui_Pdh0 && INFERENCE_PROB_MASK) {
            continue;
        } else if (packet->resListDataArray[i].ui_Pdh0 && SIDELOBE_PROB_MASK) {
            continue;
        }

        data.Pdh0          = packet->resListDataArray[i].ui_Pdh0;

        data.AzAng0        = packet->resListDataArray[i].f_AzAng0;
        data.RCS0          = packet->resListDataArray[i].f_RCS0;
        data.AzAngVar0     = packet->resListDataArray[i].f_AzAngVar0;
        data.Prob0         = packet->resListDataArray[i].f_Prob0;

        data.AzAng1        = packet->resListDataArray[i].f_AzAng1;
        data.RCS1          = packet->resListDataArray[i].f_RCS1;
        data.AzAngVar1     = packet->resListDataArray[i].f_AzAngVar1;
        data.Prob1         = packet->resListDataArray[i].f_Prob1;

        // Sanity check to make sure that assumption0 is indeed the better assumption
        assert(data.Prob0 >= data.Prob1);

        data.VrelRad      = packet->resListDataArray[i].f_VrelRad;
        //Told to ignore ElAng by continental, but store anyways since we parsed it
        data.ElAng        = packet->resListDataArray[i].f_ElAng;
        data.RangeVar     = packet->resListDataArray[i].f_RangeVar;
        data.VrelRadVar   = packet->resListDataArray[i].f_VrelRadVar;
        data.ElAngVar     = packet->resListDataArray[i].f_ElAngVar;
        data.SNR          = packet->resListDataArray[i].f_SNR;
        data.Range        = packet->resListDataArray[i].f_Range;

        // Updated calculations based on email thread with Conti
        // AzAng0 should always be the better of the two assupmtions
        data.posX = packet->resListDataArray[i].f_Range * cos(data.AzAng0);
        data.posY = packet->resListDataArray[i].f_Range * sin(data.AzAng0);
        data.posZ = 0.0;  // Z really should not be in your interface.

        msg->Detections.push_back(data);
    }
}

// Currently unused
void loadSSMessage(SSPacket_t* packet, radar_driver::SensorStatus* msg) {
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

