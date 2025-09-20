/******************************************************************************
    Filename: hal_rf.c

    This file contains functions for accessing the CC1100/CC2500 family
    of RF ICs from Texas Instruments.

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include "driverlib.h"
#include "cc110L.h"
#include "hal_rf.h"
#include "main.h"

#define CC1101_BURST_LENGTH 47
#define CC110L_BURST_LENGTH 47

/* dummy comment for test */
/*
// Rf settings for CC1101 - 4.8k baud; manual calibration
const HAL_RF_BURST_CONFIG myRfConfigCC1101 = {
  0x09,  // IOCFG2              GDO2 Output Pin Configuration - CCA indicator
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x06,  // IOCFG0              GDO0 Output Pin Configuration
  0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3D,  // PKTLEN              Packet Length
  0x05,  // PKTCTRL1            Packet Automation Control
  0x05,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x00,  // CHANNR              Channel Number
  0x06,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x23,  // FREQ2               Frequency Control Word, High Byte
  0x31,  // FREQ1               Frequency Control Word, Middle Byte
  0x3B,  // FREQ0               Frequency Control Word, Low Byte
  0xC7,  // MDMCFG4             Modem Configuration
  0x83,  // MDMCFG3             Modem Configuration
  0x13,  // MDMCFG2             Modem Configuration
  0x22,  // MDMCFG1             Modem Configuration
  0xF8,  // MDMCFG0             Modem Configuration
  0x40,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x10,  // MCSM1               Main Radio Control State Machine Configuration - CCA - RSSI below threshold
  0x08,  // MCSM0               Main Radio Control State Machine Configuration
  0x16,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  0x43,  // AGCCTRL2            AGC Control
  0x40,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xFB,  // WORCTRL             Wake On Radio Control
  0x56,  // FREND1              Front End RX Configuration
  0x10,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x7F,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x81,  // TEST2               Various Test Settings
  0x35,  // TEST1               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};
*/

// Rf settings for CC1101 - 38.4k baud; manual calibration
const HAL_RF_BURST_CONFIG myRfConfigCC1101 = {
  0x09,  // IOCFG2              GDO2 Output Pin Configuration - CCA indicator
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x06,  // IOCFG0              GDO0 Output Pin Configuration
  0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3D,  // PKTLEN              Packet Length
  0x05,  // PKTCTRL1            Packet Automation Control
  0x05,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x00,  // CHANNR              Channel Number
  0x06,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x23,  // FREQ2               Frequency Control Word, High Byte
  0x31,  // FREQ1               Frequency Control Word, Middle Byte
  0x3B,  // FREQ0               Frequency Control Word, Low Byte
  0xCA,  // MDMCFG4             Modem Configuration
  0x83,  // MDMCFG3             Modem Configuration
  0x13,  // MDMCFG2             Modem Configuration
  0x22,  // MDMCFG1             Modem Configuration
  0xF8,  // MDMCFG0             Modem Configuration
  0x35,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x10,  // MCSM1               Main Radio Control State Machine Configuration - CCA - RSSI below threshold
  0x08,  // MCSM0               Main Radio Control State Machine Configuration
  0x16,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  0x43,  // AGCCTRL2            AGC Control
  0x40,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xFB,  // WORCTRL             Wake On Radio Control
  0x56,  // FREND1              Front End RX Configuration
  0x10,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x7F,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x81,  // TEST2               Various Test Settings
  0x35,  // TEST1               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};

// Rf settings for CC110L

const HAL_RF_BURST_CONFIG myRfConfigCC110L = {
  0x09,  // IOCFG2             GDO2 Output Pin Configuration
  0x2E,  // IOCFG1             GDO1 Output Pin Configuration
  0x06,  // IOCFG0             GDO0 Output Pin Configuration
  0x47,  // FIFOTHR            RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1              Sync Word, High Byte
  0x91,  // SYNC0              Sync Word, Low Byte
  0x3D,  // PKTLEN             Packet Length
  0x05,  // PKTCTRL1           Packet Automation Control
  0x05,  // PKTCTRL0           Packet Automation Control
  0x00,  // ADDR               Device Address
  0x00,  // CHANNR             Channel number
  0x06,  // FSCTRL1            Frequency Synthesizer Control
  0x00,  // FSCTRL0            Frequency Synthesizer Control
  0x21,  // FREQ2              Frequency Control Word, High Byte
  0xE3,  // FREQ1              Frequency Control Word, Middle Byte
  0x8E,  // FREQ0              Frequency Control Word, Low Byte
  0xCA,  // MDMCFG4            Modem Configuration
  0x75,  // MDMCFG3            Modem Configuration
  0x13,  // MDMCFG2            Modem Configuration
  0x22,  // MDMCFG1            Modem Configuration
  0xE5,  // MDMCFG0            Modem Configuration
  0x34,  // DEVIATN            Modem Deviation Setting
  0x07,  // MCSM2              Main Radio Control State Machine Configuration
  0x10,  // MCSM1              Main Radio Control State Machine Configuration
  0x08,  // MCSM0              Main Radio Control State Machine Configuration
  0x16,  // FOCCFG             Frequency Offset Compensation Configuration
  0x6C,  // BSCFG              Bit Synchronization Configuration
  0x43,  // AGCCTRL2           AGC Control
  0x40,  // AGCCTRL1           AGC Control
  0x91,  // AGCCTRL0           AGC Control
  0x00,  // dummy
  0x00,  // dummy
  0xFB,  // RESERVED_0X20      Use setting from SmartRF Studio
  0x56,  // FREND1             Front End RX Configuration
  0x10,  // FREND0             Front End TX Configuration
  0xE9,  // FSCAL3             Frequency Synthesizer Calibration
  0x2A,  // FSCAL2             Frequency Synthesizer Calibration
  0x00,  // FSCAL1             Frequency Synthesizer Calibration
  0x1F,  // FSCAL0             Frequency Synthesizer Calibration
  0x00,  // dummy
  0x00,  // dummy
  0x59,  // RESERVED_0X29      Use setting from SmartRF Studio
  0x7F,  // RESERVED_0X2A      Use setting from SmartRF Studio
  0x3F,  // RESERVED_0X2B      Use setting from SmartRF Studio
  0x81,  // TEST2              Various Test Settings
  0x35,  // TEST1              Various Test Settings
  0x09,  // TEST0              Various Test Settings
};
//----------------------------------------------------------------------------------
//  void halRfResetChip(void)
//
//  DESCRIPTION:
//    Resets the chip using the procedure described in the datasheet.
//----------------------------------------------------------------------------------
void halRfResetChip(void)
{
    // Toggle chip select signal
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    myDelay(1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    myDelay(1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    myDelay(1);

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    myDelay(2);

    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6));  // wait until MISO pin is low
    // Send SRES command
    halSpiStrobe(CC110L_SRES);
    // now wait for status
    myDelay(1);
    //while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == GPIO_INPUT_PIN_HIGH);  // wait until MISO pin is low

}


//----------------------------------------------------------------------------------
//  void  halRfBurstConfig(const HAL_RF_BURST_CONFIG rfConfig, const uint8_t* rfPaTable, uint8_t rfPaTableLen)
//
//  DESCRIPTION:
//    Used to configure all of the CC1100/CC2500 registers in one burst write.
//
//  ARGUMENTS:
//    rfConfig     - register settings
//    rfPaTable    - array of PA table values (from SmartRF Studio)
//    rfPaTableLen - length of PA table
//
//----------------------------------------------------------------------------------
void  halRfBurstConfig(void)
{

    // break config registers up so as to not write in "not used" or "reserved" locations
    halSpiWrite(CC110L_IOCFG2  | CC110L_WRITE_BURST, myRfConfigCC110L, CC110L_BURST_LENGTH);

    uint8_t rfPaTable = 0x8E;  // 0dBm
    halRfWriteReg(CC110L_PATABLE,rfPaTable);
}

//----------------------------------------------------------------------------------
//  uint8_t halRfGetChipId(void)
//----------------------------------------------------------------------------------
uint8_t halRfGetChipId(void)
{
    return(halRfReadStatusReg(CC110L_PARTNUM));
}

//----------------------------------------------------------------------------------
//  uint8_t halRfGetChipVer(void)
//----------------------------------------------------------------------------------
uint8_t halRfGetChipVer(void)
{
    return(halRfReadStatusReg(CC110L_VERSION));
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfStrobe(uint8_t cmd)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfStrobe(uint8_t cmd)
{
    return(halSpiStrobe(cmd));
}

//----------------------------------------------------------------------------------
//  uint8_t halRfReadStatusReg(uint8_t addr)
//
//  NOTE:
//      When reading a status register over the SPI interface while the register
//      is updated by the radio hardware, there is a small, but finite, probability
//      that the result is corrupt. The CC1100 and CC110L errata notes explain the
//      problem and propose several workarounds.
//
//----------------------------------------------------------------------------------
uint8_t halRfReadStatusReg(uint8_t addr)
{
    uint8_t reg;
    halSpiRead(addr | CC110L_READ_BURST, &reg, 1);
    return(reg);
}

//----------------------------------------------------------------------------------
//  uint8_t halRfReadReg(uint8_t addr)
//----------------------------------------------------------------------------------
uint8_t halRfReadReg(uint8_t addr)
{
    uint8_t reg;
    halSpiRead(addr | CC110L_READ_SINGLE, &reg, 1);
    return(reg);
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfWriteReg(uint8_t addr, uint8_t data)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfWriteReg(uint8_t addr, uint8_t data)
{
    uint8_t rc;
    rc = halSpiWrite(addr, &data, 1);
    return(rc);
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfWriteFifo(uint8_t* data, uint8_t length)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfWriteFifo(const uint8_t* data, uint8_t length)
{
    return(halSpiWrite(CC110L_TXFIFO | CC110L_WRITE_BURST, data, length));
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfReadFifo(uint8_t* data, uint8_t length)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfReadFifo(uint8_t* data, uint8_t length)
{
    return(halSpiRead(CC110L_RXFIFO | CC110L_READ_BURST, data, length));
}

//----------------------------------------------------------------------------------
//  uint8_t halRfGetTxStatus(void)
//
//  DESCRIPTION:
//      This function transmits a No Operation Strobe (SNOP) to get the status of
//      the radio and the number of free bytes in the TX FIFO
//
//      Status byte:
//
//      ---------------------------------------------------------------------------
//      |          |            |                                                 |
//      | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
//      |          |            |                                                 |
//      ---------------------------------------------------------------------------
//
//  NOTE:
//      When reading a status register over the SPI interface while the register
//      is updated by the radio hardware, there is a small, but finite, probability
//      that the result is corrupt. This also applies to the chip status byte. The
//      CC1100 and CC110L errata notes explain the problem and propose several
//      workarounds.
//
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfGetTxStatus(void)
{
    return(halSpiStrobe(CC110L_SNOP));
}

//----------------------------------------------------------------------------------
//  uint8_t halRfGetRxStatus(void)
//
//  DESCRIPTION:
//      This function transmits a No Operation Strobe (SNOP) with the read bit set
//      to get the status of the radio and the number of available bytes in the RX
//      FIFO.
//
//      Status byte:
//
//      --------------------------------------------------------------------------------
//      |          |            |                                                      |
//      | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
//      |          |            |                                                      |
//      --------------------------------------------------------------------------------
//
//  NOTE:
//      When reading a status register over the SPI interface while the register
//      is updated by the radio hardware, there is a small, but finite, probability
//      that the result is corrupt. This also applies to the chip status byte. The
//      CC1100 and CC110L errata notes explain the problem and propose several
//      workarounds.
//
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfGetRxStatus(void)
{
    return(halSpiStrobe(CC110L_SNOP | CC110L_READ_SINGLE));
}


//----------------------------------------------------------------------------------
//  void halRfWakeUpChip(void)
//
//  DESCRIPTION:
//    Wakes up the chip using the procedure described in the datasheet.
//----------------------------------------------------------------------------------
void halRfWakeUpChip(void)
{
    uint8_t status;

    // Set CSN low to wake up chip
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    myDelay(1);  // guess delay - need to adjust

    do {
        status = halRfGetChipId();  // dummy read to check the chip ready bit
    } while (status & 0x80);

    // restore TEST0 register
    halRfWriteReg(CC110L_TEST0, myRfConfigCC110L[CC110L_TEST0]);
}

void halRfRadioSleep(void)
{
    halRfStrobe(CC110L_SIDLE);  // go to idle state
    halRfStrobe(CC110L_SPWD);  // put radio to sleep
}
