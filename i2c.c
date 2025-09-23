/*
 * i2c.c
 *
 *  Created on: May 16, 2018
 *      Author: dmcneill
 */
#include "driverlib.h"
#include "si7021.h"
#include "functionHeader.h"
#include "main.h"

static uint8_t RXData[30];  // used by interrupt routine to hold received data
static uint8_t TXData[30];
static uint8_t byteCount;
static uint8_t I2CMode;  // indicate whether I2C is sending or receiving


// This routine will measure the temperature, humidity and pressure - all on the I2C bus
void measureI2C(uint8_t includePressure)
{
    extern struct sysResults wxOut;
    extern struct sysResultsTime wxOutTime;
    extern uint32_t epochTime;

    // enable I2C by configuring the pins
    enableI2C();

    // first send command to start pressure measurement
    if (includePressure)
    {
        // put sensor in one shot mode
        TXData[0] = LPS35HW_CTRL_REG1;
        TXData[1] = LPS35HW_ONE_SHOT_MODE;
        byteCount = 2;
        I2CSendCommand(LPS35HW_ADDRESS);

        // now start measurement
        TXData[0] = LPS35HW_CTRL_REG2;
        TXData[1] = LPS35HW_MEASURE_PRESSURE;
        byteCount = 2;
        I2CSendCommand(LPS35HW_ADDRESS);
    }

    // send command to measure humidity and temperature
    TXData[0] = HDC3022_MEASURE_MSB;
    TXData[1] = HDC3022_MEASURE_LSB;
    byteCount = 2;
    I2CSendCommand(HDC3022_ADDRESS);

    // need to wait 10ms for measurement
    myDelay(180); // delay about 15ms

    // now receive humidity and temperature results
    byteCount = 6; // two bytes for humidity plus one for CRC; same for temperature
    I2CReceiveCommand(HDC3022_ADDRESS);
    // now get results
    wxOut.temperature = (RXData[0] << 8) + RXData[1];
    wxOutTime.temperature = epochTime;
    wxOut.humidity = (RXData[3] << 8) + RXData[4];
    wxOutTime.humidity = epochTime;

    // now get the pressure measurement results
    if (includePressure)
    {
        // verify that measurement is complete
        do {
            byteCount = 1;
            TXData[0] = LPS35HW_CTRL_REG2;
            I2CSendCommand(LPS35HW_ADDRESS);
            byteCount = 1;
            I2CReceiveCommand(LPS35HW_ADDRESS);
        } while (RXData[0] != LPS35HW_MEASURE_COMPLETE);

        // now get completed results
        byteCount = 1;
        TXData[0] = LPS35HW_PRESS_OUT;
        I2CSendCommand(LPS35HW_ADDRESS);
        byteCount = 3;
        I2CReceiveCommand(LPS35HW_ADDRESS);
        wxOut.pressure = (((uint32_t)RXData[2]) << 16) + (((uint32_t)RXData[1]) << 8) + ((uint32_t)RXData[0]);
        wxOutTime.pressure = epochTime;
        // for LPS35HW, don't think this is needed below, so comment out for now
        //initPressure();  // power down pressure sensor
    }
    disableI2C();
}

#pragma vector=USCI_B0_VECTOR
__interrupt
void USCIB0_ISR(void)
{
    static uint8_t count = 0;
    switch(__even_in_range(UCB0IV,0x1E))
    {
    case 0x00: break;       // Vector 0: No interrupts break;
    case 0x02: break;       // Vector 2: ALIFG break;
    case 0x04:
        if (I2CMode == I2C_RECEIVE) {
            EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
        } else {
            EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
        }
        break;     // Vector 4: NACKIFG break;
    case 0x06: break;       // Vector 6: STT IFG break;
    case 0x08: break;       // Vector 8: STPIFG break;
    case 0x0a: break;       // Vector 10: RXIFG3 break;
    case 0x0c: break;       // Vector 14: TXIFG3 break;
    case 0x0e: break;       // Vector 16: RXIFG2 break;
    case 0x10: break;       // Vector 18: TXIFG2 break;
    case 0x12: break;       // Vector 20: RXIFG1 break;
    case 0x14: break;       // Vector 22: TXIFG1 break;
    case 0x16:
        RXData[count++] = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);   // Get RX data
        if ( count == byteCount) {
            count = 0;
            __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
        }
        break;     // Vector 24: RXIFG0 break;
    case 0x18:
        if (++count < byteCount)                    // Check TX byte counter
        {
            EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE,TXData[count] );
        }
        else
        {
            EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
            count = 0;
            __bic_SR_register_on_exit(LPM0_bits);// Exit LPM0
        }

        break;       // Vector 26: TXIFG0 break;
    case 0x1a: break;           // Vector 28: BCNTIFG break;
    case 0x1c: break;       // Vector 30: clock low timeout break;
    case 0x1e: break;       // Vector 32: 9th bit break;
    default: break;
    }
}


// This routine is to shut down the i2c block for the si7021 and bmp280
static void disableI2C(void)
{
    EUSCI_B_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT + EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT);
    EUSCI_B_I2C_disable(EUSCI_B0_BASE);   // disable I2C block

    // Set I2C I/O to input
    GPIO_setAsInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN7
        );
}

// This routine will enable the I2C pins
static void enableI2C(void)
{
    // Configure Pins for I2C
    //Set P1.6 and P1.7 as Secondary Module Function Input.
    //Select Port 1
    //Set Pin 6, 7 to input Secondary Module Function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P1,
       GPIO_PIN6 + GPIO_PIN7,
       GPIO_SECONDARY_MODULE_FUNCTION
       );

}

/**************** I2C Send Command ******************/
// This routine will send I2C data
// address is the slave address
// put data to be sent in the TXData vector
// set byteCount to the number of bytes to send
static void I2CSendCommand(uint8_t address)
{
    extern uint8_t volatile timeOut;

    I2CMode = I2C_SEND;
    // Set up I2C block for transmission
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, address);
    //Set Master in transmit mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    // set up interrupts
    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
                EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                EUSCI_B_I2C_NAK_INTERRUPT
                );
    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
                EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                EUSCI_B_I2C_NAK_INTERRUPT
              );
    // make sure stop has been sent
    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent
            (EUSCI_B0_BASE));

    // start by sending first byte
    startTimeOut();
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, TXData[0]);
    // now sleep while I2C block sends all the data
    __bis_SR_register(LPM0_bits + GIE);
    //Delay until transmission completes before returning
    while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) && !timeOut) {;}
    stopTimeOut();
}

/**************** I2C Receive Command ******************/
// This routine will receive I2C data
// address is the slave address
// received data is put into RXData vector
// set byteCount to the number of bytes to receive
static void I2CReceiveCommand(uint8_t address)
{
    extern uint8_t volatile timeOut;

    I2CMode = I2C_RECEIVE;
    // Set up I2C block for reception
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = byteCount;
    param.autoSTOPGeneration = EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, address);
    //Set Master in receive mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    // set up interrupts
    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
        EUSCI_B_I2C_NAK_INTERRUPT
        );
    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
        EUSCI_B_I2C_NAK_INTERRUPT
        );

    // make sure stop has been sent
    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent
            (EUSCI_B0_BASE));

    // start by sending first byte
    EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
    // now sleep while I2C block receives all the data
    startTimeOut();
    __bis_SR_register(LPM0_bits + GIE);
    stopTimeOut();
}

// This function will power down the pressure sensor structures
void initPressure(void)
{

    // enable I2C by configuring the pins
    enableI2C();

    // initialize by turning off sensor
    TXData[0] = LPS35HW_CTRL_REG1;
    TXData[1] = LPS35HW_POWER_DOWN;
    byteCount = 2;
    I2CSendCommand(LPS35HW_ADDRESS);

    disableI2C();
}

