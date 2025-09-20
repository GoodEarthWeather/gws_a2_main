/*
 * packet.c
 *
 *  Created on: May 29, 2018
 *      Author: David
 */

#include "main.h"

#define INFO_A 0x1980  // memory location of information memory A

uint8_t hubID = 0x01;  // address of hub; counts from bottom up
uint8_t nodeID = 0xFE; // address of this node; counts from top down

// don't initialize - use SCMD_RFId to set these
//uint8_t hubID;  // address of hub; counts from bottom up
//uint8_t nodeID; // address of this node; counts from top down

static uint8_t data[64];  // data packet; 64 is length of FIFO in radio


// This function will build a packet for sending to hub and is based on the command being sent

void buildPacket(uint8_t command)
{
    extern uint32_t epochTime;
    extern struct sysResults wxOut;
    extern struct sysResultsTime wxOutTime;
    extern int16_t rssi_dBm;
    extern uint8_t dataChannel;
    extern struct systemStatus sysState;

    switch (command)
    {
    case CMD_REQ_HT :
        data[0] = LENGTH_CMD_RTN_HT;
        data[1] = hubID;
        data[2] = CMD_RTN_HT;
        data[3] = (uint8_t)(wxOut.humidity & 0xFF); //lsb
        data[4] = (uint8_t)(wxOut.humidity >> 8); //msb
        data[5] = (uint8_t)(wxOut.temperature & 0xFF); //lsb
        data[6] = (uint8_t)(wxOut.temperature >> 8); //msb
        data[7] = (uint8_t)(wxOutTime.temperature & 0xFF); //lsb
        data[8] = (uint8_t)(wxOutTime.temperature >> 8);
        data[9] = (uint8_t)(wxOutTime.temperature >> 16);
        data[10] = (uint8_t)(wxOutTime.temperature >> 24); //msb
        break;
    case CMD_REQ_HTP :
        data[0] = LENGTH_CMD_RTN_HTP;
        data[1] = hubID;
        data[2] = CMD_RTN_HTP;
        data[3] = (uint8_t)(wxOut.humidity & 0xFF); //lsb
        data[4] = (uint8_t)(wxOut.humidity >> 8); //msb
        data[5] = (uint8_t)(wxOut.temperature & 0xFF); //lsb
        data[6] = (uint8_t)(wxOut.temperature >> 8); //msb
        data[7] = (uint8_t)(wxOutTime.temperature & 0xFF); //lsb
        data[8] = (uint8_t)(wxOutTime.temperature >> 8);
        data[9] = (uint8_t)(wxOutTime.temperature >> 16);
        data[10] = (uint8_t)(wxOutTime.temperature >> 24); //msb
        data[11] = (uint8_t)(wxOut.pressure & 0xFF); //lsb
        data[12] = (uint8_t)(wxOut.pressure >> 8);
        data[13] = (uint8_t)(wxOut.pressure >> 16);
        data[14] = (uint8_t)(wxOut.pressure >> 24); //msb
        data[15] = (uint8_t)(wxOutTime.pressure & 0xFF); //lsb
        data[16] = (uint8_t)(wxOutTime.pressure >> 8);
        data[17] = (uint8_t)(wxOutTime.pressure >> 16);
        data[18] = (uint8_t)(wxOutTime.pressure >> 24); //msb
        break;

    case CMD_REQ_WIND :
        data[0] = LENGTH_CMD_RTN_WIND;
        data[1] = hubID;
        data[2] = CMD_RTN_WIND;
        data[3] = (uint8_t)(wxOut.windCount & 0xFF); //lsb
        data[4] = (uint8_t)(wxOut.windCount >> 8); //msb
        data[5] = (uint8_t)(wxOut.gustCount & 0xFF); //lsb
        data[6] = (uint8_t)(wxOut.gustCount >> 8); //msb
        data[7] = (uint8_t)(wxOut.windDirectionCode & 0xFF); //lsb
        data[8] = (uint8_t)(wxOut.windDirectionCode >> 8); //msb
        data[9] = (uint8_t)(wxOut.gustDirectionCode & 0xFF); //lsb
        data[10] = (uint8_t)(wxOut.gustDirectionCode >> 8); //msb
        data[11] = (uint8_t)(wxOutTime.windCount & 0xFF); //lsb
        data[12] = (uint8_t)(wxOutTime.windCount >> 8);
        data[13] = (uint8_t)(wxOutTime.windCount >> 16);
        data[14] = (uint8_t)(wxOutTime.windCount >> 24); //msb
        data[15] = (uint8_t)(wxOutTime.gustCount & 0xFF); //lsb
        data[16] = (uint8_t)(wxOutTime.gustCount >> 8);
        data[17] = (uint8_t)(wxOutTime.gustCount >> 16);
        data[18] = (uint8_t)(wxOutTime.gustCount >> 24); //msb
        data[19] = (uint8_t)(wxOutTime.windDirection & 0xFF); //lsb
        data[20] = (uint8_t)(wxOutTime.windDirection >> 8);
        data[21] = (uint8_t)(wxOutTime.windDirection >> 16);
        data[22] = (uint8_t)(wxOutTime.windDirection >> 24); //msb
        data[23] = (uint8_t)(wxOut.windDirectionReferenceCode & 0xFF); //lsb
        data[24] = (uint8_t)(wxOut.windDirectionReferenceCode >> 8); //msb
        data[25] = (uint8_t)(wxOut.gustDirectionReferenceCode & 0xFF); //lsb
        data[26] = (uint8_t)(wxOut.gustDirectionReferenceCode >> 8); //msb


        break;
    case CMD_REQ_RAIN :
        data[0] = LENGTH_CMD_RTN_RAIN;
        data[1] = hubID;
        data[2] = CMD_RTN_RAIN;
        data[3] = (uint8_t)(wxOut.rainCount);
        data[4] = (uint8_t)(wxOutTime.rainCount & 0xFF); //lsb
        data[5] = (uint8_t)(wxOutTime.rainCount >> 8);
        data[6] = (uint8_t)(wxOutTime.rainCount >> 16);
        data[7] = (uint8_t)(wxOutTime.rainCount >> 24); //msb

        break;
    case CMD_REQ_POWER :
        data[0] = LENGTH_CMD_RTN_POWER;
        data[1] = hubID;
        data[2] = CMD_RTN_POWER;
        data[3] = (uint8_t)(wxOut.primaryBattery & 0xFF); //lsb
        data[4] = (uint8_t)(wxOut.primaryBattery >> 8); //msb
        data[5] = (uint8_t)(wxOut.secondaryBattery & 0xFF); //lsb
        data[6] = (uint8_t)(wxOut.secondaryBattery >> 8); //msb
        data[7] = (uint8_t)(wxOut.vbatOK & 0xFF); //lsb
        data[8] = (uint8_t)(wxOut.vbatOK >> 8); //msb
        data[9] = (uint8_t)(wxOutTime.battery & 0xFF); //lsb
        data[10] = (uint8_t)(wxOutTime.battery >> 8);
        data[11] = (uint8_t)(wxOutTime.battery >> 16);
        data[12] = (uint8_t)(wxOutTime.battery >> 24); //msb

        break;
    case CMD_REQ_NODE_ETIME :
        data[0] = LENGTH_CMD_RTN_NODE_ETIME;
        data[1] = hubID;
        data[2] = CMD_RTN_NODE_ETIME;
        data[3] = (uint8_t)(epochTime & 0xFF); //lsb
        data[4] = (uint8_t)(epochTime >> 8);
        data[5] = (uint8_t)(epochTime >> 16);
        data[6] = (uint8_t)(epochTime >> 24); //msb
        break;

    case CMD_REQ_SIMPLE :
        data[0] = LENGTH_CMD_RTN_SIMPLE;
        data[1] = hubID;
        data[2] = CMD_RTN_SIMPLE;
        data[3] = (uint8_t)(wxOut.humidity & 0xFF); //lsb
        data[4] = (uint8_t)(wxOut.humidity >> 8); //msb
        data[5] = (uint8_t)(wxOut.temperature & 0xFF); //lsb
        data[6] = (uint8_t)(wxOut.temperature >> 8); //msb
        data[7] = (uint8_t)(wxOut.windCount & 0xFF); //lsb
        data[8] = (uint8_t)(wxOut.windCount >> 8); //msb
        data[9] = (uint8_t)(wxOut.gustCount & 0xFF); //lsb
        data[10] = (uint8_t)(wxOut.gustCount >> 8); //msb
        data[11] = (uint8_t)(wxOut.windDirectionCode & 0xFF); //lsb
        data[12] = (uint8_t)(wxOut.windDirectionCode >> 8); //msb
        data[13] = (uint8_t)(wxOut.gustDirectionCode & 0xFF); //lsb
        data[14] = (uint8_t)(wxOut.gustDirectionCode >> 8); //msb
        data[15] = (uint8_t)(wxOut.pressure & 0xFF); //lsb
        data[16] = (uint8_t)(wxOut.pressure >> 8);
        data[17] = (uint8_t)(wxOut.pressure >> 16);
        data[18] = (uint8_t)(wxOut.pressure >> 24); //msb
        data[19] = (uint8_t)(wxOut.rainCount);
        data[20] = (uint8_t)(epochTime & 0xFF); //lsb
        data[21] = (uint8_t)(epochTime >> 8);
        data[22] = (uint8_t)(epochTime >> 16);
        data[23] = (uint8_t)(epochTime >> 24); //msb
        data[24] = (uint8_t)(wxOut.primaryBattery & 0xFF); //lsb
        data[25] = (uint8_t)(wxOut.primaryBattery >> 8); //msb
        data[26] = (uint8_t)(wxOut.secondaryBattery & 0xFF); //lsb
        data[27] = (uint8_t)(wxOut.secondaryBattery >> 8); //msb
        data[28] = (uint8_t)(wxOut.vbatOK & 0xFF); //lsb
        data[29] = (uint8_t)(wxOut.vbatOK >> 8); //msb
        break;

    case CMD_NODE_READY :
        data[0] = LENGTH_CMD_NODE_READY;
        data[1] = hubID;
        data[2] = CMD_NODE_READY;
        data[3] = nodeID;
        data[4] = dataChannel;

        break;
    case CMD_REQ_PRESSURE :
        data[0] = LENGTH_CMD_RTN_PRESSURE;
        data[1] = hubID;
        data[2] = CMD_RTN_PRESSURE;
        data[3] = (uint8_t)(wxOut.pressure & 0xFF); //lsb
        data[4] = (uint8_t)(wxOut.pressure >> 8);
        data[5] = (uint8_t)(wxOut.pressure >> 16);
        data[6] = (uint8_t)(wxOut.pressure >> 24); //msb
        data[7] = (uint8_t)(wxOutTime.pressure & 0xFF); //lsb
        data[8] = (uint8_t)(wxOutTime.pressure >> 8);
        data[9] = (uint8_t)(wxOutTime.pressure >> 16);
        data[10] = (uint8_t)(wxOutTime.pressure >> 24); //msb

        break;
    case CMD_REQ_NODE_RSSI :
        data[0] = LENGTH_CMD_RTN_NODE_RSSI;
        data[1] = hubID;
        data[2] = CMD_RTN_NODE_RSSI;
        data[3] = (uint8_t)(rssi_dBm & 0xFF); //lsb
        data[4] = (uint8_t)(rssi_dBm >> 8);
        data[5] = 0; // used to be lqi
        break;

    case CMD_ACK :
        data[0] = LENGTH_CMD_ACK;
        data[1] = hubID;
        data[2] = CMD_ACK;
        break;

    case CMD_REQ_NODE_SYS :
        data[0] = LENGTH_CMD_RTN_NODE_SYS;
        data[1] = hubID;
        data[2] = CMD_RTN_NODE_SYS;
        data[3] = (uint8_t)(sysState.timeOutCount & 0xFF); //lsb
        data[4] = (uint8_t)(sysState.timeOutCount >> 8);
        data[5] = (uint8_t)(sysState.upTimer & 0xFF); //lsb
        data[6] = (uint8_t)(sysState.upTimer >> 8);
        data[7] = (uint8_t)(sysState.upTimer >> 16);
        data[8] = (uint8_t)(sysState.upTimer >> 24); //msb
        data[9] = (uint8_t)(sysState.pktRXCount & 0xFF); //lsb
        data[10] = (uint8_t)(sysState.pktRXCount >> 8);
        data[11] = (uint8_t)(sysState.pktTXCount & 0xFF); //lsb
        data[12] = (uint8_t)(sysState.pktTXCount >> 8);
        data[13] = (uint8_t)(sysState.crcErrorCount & 0xFF); //lsb
        data[14] = (uint8_t)(sysState.crcErrorCount >> 8);
        data[15] = (uint8_t)(sysState.lengthErrorCount & 0xFF); //lsb
        data[16] = (uint8_t)(sysState.lengthErrorCount >> 8);
        data[17] = (uint8_t)(sysState.fifoErrorCount & 0xFF); //lsb
        data[18] = (uint8_t)(sysState.fifoErrorCount >> 8);
        data[19] = (uint8_t)(sysState.reTryCount & 0xFF); //lsb
        data[20] = (uint8_t)(sysState.reTryCount >> 8);
        break;

    default:
        break;
    }
}

// Function to send data packet - over radio
void sendPacket(uint8_t channel)
{
        // send over radio
        txSendPacket(data,channel);
}

// This routine will store the nodeID and hubID in information memory A
void setRFID(uint8_t node, uint8_t hub)
{
    uint8_t *infoA = (uint8_t *)INFO_A;  // pointer to info memory A

    FRAMCtl_write8(&node,infoA,1);
    FRAMCtl_write8(&hub,(infoA+1),1);
    nodeID = *infoA;
    hubID = *(infoA+1);
}

// This routine will retrieve the nodeID and hubID in information memory A
void getRFID(void)
{
    uint8_t *infoA = (uint8_t *)INFO_A;  // pointer to info memory A

    nodeID = *infoA;
    hubID = *(infoA+1);
}
