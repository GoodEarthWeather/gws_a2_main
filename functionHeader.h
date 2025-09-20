/*
 * functionHeader.h
 *
 *  Created on: May 10, 2018
 *      Author: dmcneill
 */

#ifndef FUNCTIONHEADER_H_
#define FUNCTIONHEADER_H_

void initRTC(void);
void initClocks(void);
void initGPIO(void);
void initWind(void);
void initRain(void);
void initUART(void);
void initADC(void);

void measureADC(void);
void measureI2C(uint8_t);
void measureRain(uint8_t);
void clearRain(void);
void recordWind(void);

void initPressure(void);


void startTimeOut(void);
void stopTimeOut(void);
void myDelay(uint8_t);

void buildPacket(uint8_t);
void sendPacket(uint8_t);
void setRFID(uint8_t, uint8_t);
void getRFID(void);

void executeCommand(void);

void initRadio(void);
void halBoardInit(void);
void halSpiInit(void);
uint8_t rxRecvPacket(uint8_t*, uint8_t);
uint8_t txSendPacket(uint8_t*, uint8_t);
void getRSSI(void);
 void getNextChannel(void);

void  halSpiInit(void);
uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint8_t len);
uint8_t halSpiWrite(uint8_t addr, const uint8_t* data, uint8_t len);
uint8_t halSpiStrobe(uint8_t cmd);


#endif /* FUNCTIONHEADER_H_ */
