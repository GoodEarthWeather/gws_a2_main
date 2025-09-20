#include "driverlib.h"

void startTimeOut(void);
void stopTimeOut(void);

uint8_t volatile timeOut;  // flag for setting timeout

void startTimeOut(void)
{

    // Using WDT in interval mode; for now assume 32768 Hz clock and 8192 divider for 250ms timeout
    timeOut = 0;  // clear flag

    //Initialize WDT module in timer interval mode,
       //with SMCLK as source
       WDT_A_initIntervalTimer(WDT_A_BASE,
           WDT_A_CLOCKSOURCE_ACLK,
           WDT_A_CLOCKDIVIDER_8192);

       WDT_A_start(WDT_A_BASE);
       //Enable Watchdog Interupt
       SFR_clearInterrupt(SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);
       SFR_enableInterrupt(SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);

}

void stopTimeOut(void)
{
    WDT_A_hold(WDT_A_BASE);
}

//Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(WDT_VECTOR)))
#endif
void WDT_A_ISR (void)
{
    timeOut = 1;  // set timeOut flag
    WDT_A_hold(WDT_A_BASE);
    __bic_SR_register_on_exit(LPM0_bits); // wakeup
}

