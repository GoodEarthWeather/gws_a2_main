/*
 * rain.c
 *
 *  Created on: May 10, 2018
 *      Author: dmcneill
 */
#include "main.h"

static uint16_t rainCount;

// initialize rain gauge I/O
void initRain(void)
{
    // Uses P1.3
    // select for interrupt on rising edge
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);

    rainCount = 0;
}

// save rain counter value and clear
void measureRain(uint8_t clearRainCount)
{
    extern struct sysResults wxOut;
    extern struct sysResultsTime wxOutTime;
    extern uint32_t epochTime;

    wxOut.rainCount = rainCount;
    wxOutTime.rainCount = epochTime;
    if (clearRainCount)
    {
        rainCount = 0;
    }
}

// clear rain counter
void clearRain(void)
{
    rainCount = 0;
}


//******************************************************************************
//
//This is the PORT1_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT1_VECTOR)))
#endif
void Port_1 (void)
{
    switch(__even_in_range(P1IV,16))
    {
      case  0: break;                         // Vector  0:  No interrupt
      case  2: break;                         // Vector  2:  Port 1 Bit 0
      case  4: break;                         // Vector  4:  Port 1 Bit 1
      case  6: break;                         // Vector  6:  Port 1 Bit 2
      case  8:
          rainCount++;
          GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
          break;                         // Vector  8:  Port 1 Bit 3
      case  10: break;                         // Vector  10:  Port 1 Bit 4
      case  12: break;                         // Vector  12:  Port 1 Bit 5
      case  14: break;                         // Vector  14:  Port 1 Bit 6
      case  16: break;                         // Vector  16:  Port 1 Bit 7
      default: break;
    }

}


