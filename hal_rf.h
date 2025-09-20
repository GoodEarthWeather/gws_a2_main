/***********************************************************************************
    Filename: hal_rf.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_RF_H
#define HAL_RF_H

//----------------------------------------------------------------------------------
// Type declarations
//----------------------------------------------------------------------------------


// For rapid chip configuration with a minimum of function overhead.
// The array has to be set up with predefined values for all registers.
typedef uint8_t HAL_RF_BURST_CONFIG[47];

// The chip status byte, returned by chip for all SPI accesses
typedef uint8_t HAL_RF_STATUS;



//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

//void  halRfConfig(const HAL_RF_CONFIG* rfConfig, const uint8_t* rfPaTable, uint8_t rfPaTableLen);
void  halRfConfig(uint8_t unitID);
void  halRfBurstConfig(void);
void  halRfResetChip(void);
uint8_t halRfGetChipId(void);
uint8_t halRfGetChipVer(void);
uint8_t halRfReadStatusReg(uint8_t addr);
uint8_t halRfReadReg(uint8_t addr);
void halRfWakeUpChip(void);
void halRfRadioSleep(void);

HAL_RF_STATUS halRfWriteReg(uint8_t addr, uint8_t data);
HAL_RF_STATUS halRfWriteFifo(const uint8_t* data, uint8_t length);
HAL_RF_STATUS halRfReadFifo(uint8_t* data, uint8_t length);
HAL_RF_STATUS halRfStrobe(uint8_t cmd);
HAL_RF_STATUS halRfGetTxStatus(void);
HAL_RF_STATUS halRfGetRxStatus(void);


/**********************************************************************************/
#endif
