/*
 * main.h
 *
 *  Created on: May 16, 2018
 *      Author: dmcneill
 */

#ifndef MYINCLUDE_MAIN_H_
#define MYINCLUDE_MAIN_H_

#define SYNC_RADIO_CHANNEL 0

#include "driverlib.h"
#include "functionHeader.h"
#include "command.h"

struct sysResults {
    uint16_t humidity;
    uint16_t temperature;
    uint32_t pressure;
    uint16_t windCount;
    uint16_t gustCount;
    uint16_t windDirectionCode;
    uint16_t windDirectionReferenceCode;
    uint16_t gustDirectionCode;
    uint16_t gustDirectionReferenceCode;
    uint8_t rainCount;
    uint16_t primaryBattery;
    uint16_t secondaryBattery;
    uint16_t vbatOK;
    uint16_t groundReference;
    uint16_t checksum;
};

struct sysResultsTime {
    uint32_t humidity;
    uint32_t temperature;
    uint32_t pressure;
    uint32_t windCount;
    uint32_t gustCount;
    uint32_t windDirection;
    uint32_t gustDirection;
    uint32_t rainCount;
    uint32_t battery;
};

struct systemStatus {
    uint16_t timeOutCount;  // number of timeouts
    uint32_t upTimer;    // counter value since power up
    uint16_t pktRXCount;   // number of valid packets received
    uint16_t pktTXCount;   // number of packets sent
    uint16_t crcErrorCount;  // number of packets with crc error
    uint16_t lengthErrorCount;  // number of packets with packet length error
    uint16_t fifoErrorCount;  // number of packets with fifo over/unflow
    uint16_t reTryCount;  // number of times to re-send packets that had errors
};

struct windDirection {
    uint16_t directionCode;
    uint16_t referenceCode;
};
void measureWindDirection(struct windDirection *);

#endif /* MYINCLUDE_MAIN_H_ */
