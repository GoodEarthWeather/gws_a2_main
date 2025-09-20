/*
 * myRadio.c
 *
 *  Created on: May 31, 2018
 *      Author: David
 */

#include "driverlib.h"
#include "cc110L.h"
#include "hal_rf.h"
#include "main.h"

//----------------------------------------------------------------------------------
//  Constants used in this file
//----------------------------------------------------------------------------------

#define RX_OK                0
#define RX_LENGTH_VIOLATION  1
#define RX_CRC_MISMATCH      2
#define RX_FIFO_OVERFLOW     3
#define RX_TIMEOUT  4

#define RADIO_MODE_TX        1
#define RADIO_MODE_RX        2


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
volatile uint8_t radioMode;
volatile uint8_t packetSent;
volatile uint8_t packetReceived;
int16_t rssi_dBm;
uint8_t dataChannel;
extern struct systemStatus sysState;

static void cca(void);

typedef uint8_t RF_DATA_CHANNELS[50];
static const RF_DATA_CHANNELS dataChannelList = {
 27, 32, 50, 37, 28, 8, 35, 10,
 46, 20, 45, 25, 38, 12, 1, 33, 41, 14, 49, 24, 11, 23, 16, 47, 9,
 36, 2, 34, 5, 19, 15, 39, 4, 48, 3, 7, 44,
 30, 17, 18, 40, 43, 42, 31, 26, 21, 29, 22, 6, 13};

// initialize radio
void initRadio (void)
{
    extern uint8_t nodeID;

    halSpiInit();
    halRfResetChip();

    // Setup chip with register settings from SmartRF Studio
    halRfBurstConfig();  // burst configuration
    //halRfConfig(nodeID);
    // now write ID
    halRfWriteReg(CC110L_ADDR,nodeID);          //Device Address - either hubID or nodeID

    // enable interrupts on GDO0
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN6); // enable GDO0 interrupt

}

//----------------------------------------------------------------------------------
//  uint8_t txSendPacket(uint8_t* data, uint8_t length)
//
//  DESCRIPTION:
//    Send a packet that is smaller than the size of the FIFO, making it possible
//    to write the whole packet at once. Wait for the radio to signal that the packet
//    has been transmitted.
//
//  ARGUMENTS:
//    data   - Data to send. First byte contains length byte
//    length - Total length of packet to send
//
//  RETURNS:
//    This function always returns 0.
//----------------------------------------------------------------------------------
uint8_t txSendPacket(uint8_t* data, uint8_t channel)
{
    //packetSent = FALSE;
    //radioMode = RADIO_MODE_TX;
    uint8_t status;
    uint8_t i = 0;


    cca();  // wait for clear channel
    // Write data to FIFO
    halRfStrobe(CC110L_SIDLE);  // go to idle state
    halRfWriteReg(CC110L_CHANNR, channel);
    halRfStrobe(CC110L_SFTX);  // first flush the TX FIFO
    halRfWriteFifo(data, data[0]+1); // length value doesn't include the length byte - so add 1

    // Set radio in transmit mode
    packetSent = 0;
    radioMode = RADIO_MODE_TX;
    //GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN4); // enable GDO2 interrupt - PLL Lock Indicator
    halRfStrobe(CC110L_SCAL);  // calibrate synthesizer

    // wait for calibration to finish and return to idle state
    do {
        status = halSpiStrobe(CC110L_SNOP);  // get state machine state
        i++;
    } while ((status >> 4) != 0);  // wait until in the idle state

    halRfStrobe(CC110L_STX);  // goes to transmit state

    while(!packetSent)
    {
        __bis_SR_register(LPM0_bits + GIE);   // go to sleep and wait for radio to send command
    }
    sysState.pktTXCount++;
    return(0);
}

//----------------------------------------------------------------------------------
//  uint8_t rxRecvPacket(uint8_t* data, uint8_t* length)
//
//  DESCRIPTION:
//    Receive a packet that is smaller than the size of the FIFO, i.e. wait for the
//    complete packet to be received before reading from the FIFO. This function sets
//    the CC1100/CC2500 in RX and waits for the chip to signal that a packet is received.
//
//  ARGUMENTS:
//    data   - Where to write incoming data.
//    length - Length of payload.
//
//  RETURNS:
//    0 if a packet was received successfully.
//    1 if chip is in overflow state (packet longer than FIFO).
//    2 if the length of the packet is illegal (0 or > 61).
//    3 if the CRC of the packet is not OK.
//----------------------------------------------------------------------------------
uint8_t rxRecvPacket(uint8_t* data, uint8_t channel)
{
    uint8_t packet_status[2];
    uint8_t status, length;
    extern volatile uint8_t validCommand;
    extern uint8_t timeOut;


    // Set radio in RX mode
    packetReceived = 0;
    radioMode = RADIO_MODE_RX;
    halRfStrobe(CC110L_SIDLE);  // go to idle state
    halRfWriteReg(CC110L_CHANNR, channel);
    halRfStrobe(CC110L_SFRX);  // first flush the RX FIFO

    halRfStrobe(CC110L_SCAL);  // calibrate synthesizer
    // wait for calibration to finish and go back to idle state
    do {
        status = halSpiStrobe(CC110L_SNOP);  // get state machine state
    } while ((status >> 4) != 0);  // wait until in the idle state

    // now that vco is calibrated
    halRfStrobe(CC110L_SRX);  // go to receive state

    //key = halIntLock();
    while(!packetReceived  && !timeOut)
    {
        __bis_SR_register(LPM0_bits + GIE);   // go to sleep and wait for radio to receive command
    }

    if ( timeOut )
    {
        halRfStrobe(CC110L_SIDLE);
        sysState.timeOutCount++;
        return(RX_TIMEOUT);  // need to handle time out at calling function
    }
    // Read first element of packet from the RX FIFO
    // get first element (length)
    status = halRfReadFifo(&length, 1);
    data[0] = length;
    //data++;

    if ((status & CC110L_STATUS_STATE_BM) == CC110L_STATE_RX_OVERFLOW)
    {
        halRfStrobe(CC110L_SIDLE);
        halRfStrobe(CC110L_SFRX);
        status = RX_FIFO_OVERFLOW;
        sysState.fifoErrorCount++;
    }
    else if (length == 0 || length > 61)
    {
        halRfStrobe(CC110L_SIDLE);
        halRfStrobe(CC110L_SFRX);
        status = RX_LENGTH_VIOLATION;
        sysState.lengthErrorCount++;
    }
    else
    {
        // Get payload
        halRfReadFifo(data, length);  // get payload
        halRfReadFifo(packet_status,2);  // get appended bytes
        status = packet_status[1];
        if ((status & CC110L_CRC_OK_BM))  // true if no crc error
        {
            validCommand = 1;
            status = RX_OK;
            sysState.pktRXCount++;
        }
        else
        {
            // crc error
            status = RX_CRC_MISMATCH;
            sysState.crcErrorCount++;
        }
    }
    return(status);
}

// Routine to get rssi value from status register
void getRSSI(void)
{
    uint8_t rssi_dec;
    uint8_t rssi_offset = 74;

    rssi_dec = halRfReadStatusReg(CC110L_RSSI);
    if (rssi_dec >= 128)
    {
        rssi_dBm = (int16_t)((int16_t)( rssi_dec - 256) / 2) - rssi_offset;
    }
    else
    {
        rssi_dBm = (rssi_dec / 2) - rssi_offset;
    }
}

// Routine to check for CCA before transmission
static void cca(void)
{
    uint8_t status;

    // go to receiver mode
    radioMode = RADIO_MODE_RX;
    halRfStrobe(CC110L_SIDLE);  // go to idle state
    halRfStrobe(CC110L_SCAL);  // calibrate synthesizer
    // wait for calibration to finish and go back to idle state
    do {
        status = halSpiStrobe(CC110L_SNOP);  // get state machine state
    } while ((status >> 4) != 0);  // wait until in the idle state

    // now that vco is calibrated
    halRfStrobe(CC110L_SRX);  // go to receive state

    // now wait for CCA indicator to go high
    while(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN4) == GPIO_INPUT_PIN_LOW);  // wait until GDO2 pin is high, indicating RSSI below threshold

}
//******************************************************************************
//
//This is the PORT3_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT3_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT3_VECTOR)))
#endif
void Port_3 (void)
{
    switch(__even_in_range(P3IV,16))
    {
      case  0: break;                         // Vector  0:  No interrupt
      case  2: break;                         // Vector  2:  Port 3 Bit 0
      case  4: break;                         // Vector  4:  Port 3 Bit 1
      case  6: break;                         // Vector  6:  Port 3 Bit 2
      case  8: break;                         // Vector  8:  Port 3 Bit 3
      case  10:
          // interrupt here on rising edge of P3.4 = GDO2 = indication of PLL locked
          GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN4);  // clear interupt on GDO2 - p3.4
          __bic_SR_register_on_exit(LPM0_bits); // wake up
          break;                         // Vector  10:  Port 3 Bit 4
      case  12: break;                         // Vector  12:  Port 3 Bit 5
      case  14:
          if ( radioMode == RADIO_MODE_TX )  // we are in transmit mode, so interrupt means packet has been sent
          {
              packetSent = 1;
          }
          else if ( radioMode == RADIO_MODE_RX )  // we are in receive mode, so interrupt means packet has been received
          {
              packetReceived = 1;
          }
          GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN6);  // clear interupt on GDO0 - p3.6
          __bic_SR_register_on_exit(LPM0_bits); // wake up

          break;                         // Vector  14:  Port 3 Bit 6
      case  16: break;                         // Vector  16:  Port 3 Bit 7
      default: break;
    }
}

// This routine will get the next radio channel
void getNextChannel(void)
{
    static uint8_t i = 0;
    // for now just set to 50
    //dataChannel = 50;
    dataChannel = dataChannelList[i++];
    if (i >= 50)
    {
        i = 0;
    }
}


