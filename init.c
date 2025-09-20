/*
 * init.c
 *
 *  Created on: May 10, 2018
 *      Author: dmcneill
 */
#include "driverlib.h"
//This file contains the routines to initialize everything


// initialize the clock system
void initClocks(void)
{
    //Initialize external 32.768kHz clock
    CS_setExternalClockSource(32768,0);
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);

    //Set DCO frequency to 8MHz
    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_6);
    //Set ACLK = External 32.768kHz clock with frequency divider of 1
    CS_initClockSignal(CS_ACLK,CS_LFXTCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);

}

// initialize GPIO
void initGPIO(void)
{
    // Set I2C I/O to input; will be set to I2C functionality in the I2C routines
    GPIO_setAsInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN7
        );

   /*
   * Set Pin 4, 5 to input Primary Module Function, LFXT.
   * This is for configuration of the external 32.768kHz crystal
   */
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_PJ,
       GPIO_PIN4 + GPIO_PIN5,
       GPIO_PRIMARY_MODULE_FUNCTION
   );

   // P1.2 - wind speed input - set as TA0 clock
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P1,
       GPIO_PIN2,
       GPIO_SECONDARY_MODULE_FUNCTION
   );

   // P1.3 - rain gauge input
   GPIO_setAsInputPin(
       GPIO_PORT_P1,
       GPIO_PIN3
       );

   // P4.4  Wind Direction Vreference enable
   GPIO_setAsOutputPin(
       GPIO_PORT_P4,
       GPIO_PIN4
       );
   // for now, go ahead and disable shunt reference
   GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);

   // P3.1  VBATT System measurement enable
   GPIO_setAsOutputPin(
       GPIO_PORT_P3,
       GPIO_PIN1
       );
   // for now, go ahead and disable
   GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN1);

   // P1.5  VSOLAR System measurement enable
   GPIO_setAsOutputPin(
       GPIO_PORT_P1,
       GPIO_PIN5
       );
   // for now, go ahead and disable
   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);


   // configure analog measurement inputs
   // P3.0 = Wind Direction input = A12
   // P1.0 = Wind Direction reference = A0
   // P3.2 = VBATT System sense = A14
   // P1.4 = VSOLAR System Sense = A4

   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P1,
       GPIO_PIN0 + GPIO_PIN4,
       GPIO_TERNARY_MODULE_FUNCTION
   );

   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P3,
       GPIO_PIN0 + GPIO_PIN2,
       GPIO_TERNARY_MODULE_FUNCTION
   );

   // Configure all radio I/O
   // set GDO0 (=RADIO_INT=P3.6) and GDO2 (=P3.4) as inputs
   GPIO_setAsInputPin(
           GPIO_PORT_P3,
           GPIO_PIN4 + GPIO_PIN6
   );
   GPIO_selectInterruptEdge(GPIO_PORT_P3, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);  // falling edge on GDO0
   // configure chip select (P3.5)
   GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);
   GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);

   // Set test points as outputs for now, and set low
   // Test Point 2
   GPIO_setAsOutputPin(
       GPIO_PORT_P3,
       GPIO_PIN3
       );
   GPIO_setOutputLowOnPin(
       GPIO_PORT_P3,
       GPIO_PIN3
       );

   // Test Point 1
   GPIO_setAsOutputPin(
       GPIO_PORT_P2,
       GPIO_PIN1
       );
   GPIO_setOutputLowOnPin(
       GPIO_PORT_P2,
       GPIO_PIN1
       );

   // Initialize all un-used GPIO to outputs
   // Port 1 Unused
   GPIO_setAsOutputPin(
       GPIO_PORT_P1,
       GPIO_PIN1
       );
   GPIO_setOutputLowOnPin(
       GPIO_PORT_P1,
       GPIO_PIN1
       );

   // Port J Unused
   GPIO_setAsOutputPin(
       GPIO_PORT_PJ,
       GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3
       );
   GPIO_setOutputLowOnPin(
       GPIO_PORT_PJ,
       GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3
       );

   // Port 2 Unused
   GPIO_setAsOutputPin(
       GPIO_PORT_P2,
       GPIO_PIN0 + GPIO_PIN2 + GPIO_PIN3 + GPIO_PIN7
       );
   GPIO_setOutputLowOnPin(
       GPIO_PORT_P2,
       GPIO_PIN0 + GPIO_PIN2 + GPIO_PIN3 + GPIO_PIN7
       );

   /*
    * Disable the GPIO power-on default high-impedance mode to activate
    * previously configured port settings
    */
   PMM_unlockLPM5();

}
