/******************************************************************************
    Filename: hal_spi.c

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include "driverlib.h"
#include "main.h"

//----------------------------------------------------------------------------------
//   Target specific initialization of SPI interface in hal_spi_config.c
//----------------------------------------------------------------------------------
void halSpiInit(void)
{
    /////////////////////////////////////
    // Configure SCLK pin P2.4
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_SECONDARY_MODULE_FUNCTION);
    // Configure MOSI on pin P2.5
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_SECONDARY_MODULE_FUNCTION);
    // Configure MISO on pin P2.6
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);

    // Now initialize SPI Master
    EUSCI_A_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = param.clockSourceFrequency/2;
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_A_SPI_3PIN;
    EUSCI_A_SPI_initMaster(EUSCI_A1_BASE, &param);

    // Now enable the SPI module
    EUSCI_A_SPI_enable(EUSCI_A1_BASE);

    // clear and enable interrupt
    EUSCI_A_SPI_clearInterrupt(EUSCI_A1_BASE,EUSCI_A_SPI_RECEIVE_INTERRUPT);
    //EUSCI_A_SPI_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);

}

//----------------------------------------------------------------------------------
//  void halSpiWrite(uint8_t addr, const uint8_t *buffer, uint16_t length)
//
//  DESCRIPTION:
//    Write data to device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is written. For single byte write, set length to 1.
//----------------------------------------------------------------------------------
uint8_t halSpiWrite(uint8_t addr, const uint8_t* data, uint8_t length)
{
    uint16_t i;

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // set CSN low
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6));  // wait until MISO pin is low

    // send address
    // wait for transmit interrupt to be clear
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)) { ; }
    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, addr);
    while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE) == EUSCI_A_SPI_BUSY) {;}  // wait until spi operation is complete

    for (i = 0; i < length; i++)
    {
        // wait for transmit interrupt to be clear
        while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)) { ; }
        EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, data[i]);
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); // set CSN high
    return(0);

}

//----------------------------------------------------------------------------------
//  uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint16_t length)
//
//  DESCRIPTION:
//    Read data from device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is read. Note that the master device needs to write a dummy byte
//    (in this case 0) for every new byte in order to generate the clock to
//    clock out the data. For single byte read, set length to 1.
//----------------------------------------------------------------------------------
uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint8_t length)
{
    uint16_t i;
    uint8_t status;

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // set CSN low
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6));  // wait until MISO pin is low

    // write address to read from
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)) { ; }
    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, addr);
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)) { ; }
    status = EUSCI_A_SPI_receiveData(EUSCI_A1_BASE);

    for (i = 0; i < length; i++)
    {
        // wait for transmit interrupt to be clear
        while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)) { ; }
        EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, 0);  // dummy write
        data[i] = EUSCI_A_SPI_receiveData(EUSCI_A1_BASE);
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); // set CSN high
    return(status);
}


//----------------------------------------------------------------------------------
//  uint8_t halSpiStrobe(uint8_t cmd)
//
//  DESCRIPTION:
//    Special write function, writing only one byte (cmd) to the device.
//----------------------------------------------------------------------------------
uint8_t halSpiStrobe(uint8_t cmd)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // set CSN low
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6));  // wait until MISO pin is low

     // wait for transmit interrupt to be clear
     while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)) { ; }
     EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, cmd);
     while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE) == EUSCI_A_SPI_BUSY) {;}  // wait until spi operation is complete

     GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); // set CSN high

     return(0);

}

