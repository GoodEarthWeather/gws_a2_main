/*
 * myRTC.c
 *
 *  Created on: May 10, 2018
 *      Author: dmcneill
 */


#include "main.h"

volatile Calendar newTime;
uint32_t epochTime;
uint16_t const gustSampleRate = 5;  // window of time (seconds) for measuring gust speed
uint16_t const wakeUpTimePeriod = 60; // period of time that radio wakes up to check with hub
//uint16_t const wakeUpTimePeriod = 5; // for debug work, wake up every 5 seconds

void initRTC (void)
{
    Calendar currentTime;

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    //Setup Current Time for Calendar
    currentTime.Seconds    = 0x00;
    currentTime.Minutes    = 0x26;
    currentTime.Hours      = 0x13;
    currentTime.DayOfWeek  = 0x03;
    currentTime.DayOfMonth = 0x20;
    currentTime.Month      = 0x07;
    currentTime.Year       = 0x2011;

    //Initialize Calendar Mode of RTC
    /*
     * Base Address of the RTC_B
     * Pass in current time, intialized above
     * Use BCD as Calendar Register Format
     */
    RTC_B_initCalendar(RTC_B_BASE,
        &currentTime,
        RTC_B_FORMAT_BCD);

    // Set up for a 1 second interrupt
    RTC_B_definePrescaleEvent(RTC_B_BASE, RTC_B_PRESCALE_0, RTC_B_PSEVENTDIVIDER_256);
    RTC_B_definePrescaleEvent(RTC_B_BASE, RTC_B_PRESCALE_1, RTC_B_PSEVENTDIVIDER_128);

    RTC_B_clearInterrupt(RTC_B_BASE,
        RTC_B_CLOCK_READ_READY_INTERRUPT +
        RTC_B_TIME_EVENT_INTERRUPT +
        RTC_B_CLOCK_ALARM_INTERRUPT +
        RTC_B_PRESCALE_TIMER1_INTERRUPT
        );

    // Enable interrupts
    RTC_B_enableInterrupt(RTC_B_BASE, RTC_B_PRESCALE_TIMER1_INTERRUPT);

    //Start RTC Clock
    RTC_B_startClock(RTC_B_BASE);

    //Enter LPM3 mode with interrupts enabled
    //__bis_SR_register(LPM0_bits + GIE);
    //__no_operation();
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_B_ISR (void)
{
    static uint16_t secondCounter = 0;
    static uint16_t minuteCounter = 0;
    extern uint16_t currentWindValue;
    extern uint16_t currentGustValue;
    extern struct windDirection currentGustDirection;
    extern uint32_t currentGustTime;
    extern uint16_t minuteWindValue;
    extern struct systemStatus sysState;

    switch (__even_in_range(RTCIV,16)){
        case 0: break;  //No interrupts
        case 2:         //RTCRDYIFG
            break;
        case 4:         //RTCEVIFG
            break;
        case 6:         //RTCAIFG
            break;
        case 8: break;  //RT0PSIFG
        case 10:
            // interrupts here every second
            ++epochTime;
            ++minuteCounter;  // used for counting up to 60 seconds
            ++sysState.upTimer;  // time since power up
            // check for 5 second count
            if ( ++secondCounter == gustSampleRate )  // process wind gust every gustSampleRate seconds
            {
                secondCounter = 0; // reset second counter
                // check gust values
                currentWindValue = Timer_A_getCounterValue(TIMER_A0_BASE);
                minuteWindValue += currentWindValue;  // accumulate wind counts over 1 minute
                if (currentWindValue >= currentGustValue)
                {
                    currentGustValue = currentWindValue;
                    currentGustTime = epochTime;
                    measureWindDirection(&currentGustDirection);
                }
                Timer_A_clear(TIMER_A0_BASE); // clear counter
            }
            if ( minuteCounter == wakeUpTimePeriod )  // default of 60 seconds
            {
                minuteCounter = 0;
                __bic_SR_register_on_exit(LPM3_bits); // wakeup
            }
                 break; //RT1PSIFG
        case 12: break; //Reserved
        case 14: break; //Reserved
        case 16: break; //Reserved
        default: break;
    }
}

// This routine will create a delay and return when the delay is complete - about 80uS for n = 1
// Delay is computed as n*240, where 240 is the count that is about 80uS; keep n below 255
void myDelay(uint8_t n)
{

    // set test gpio high to start delay
    //GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

    // use timer A1
    //Start timer in continuous mode sourced by SMCLK
    Timer_A_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initContParam.timerClear = TIMER_A_DO_CLEAR;
    initContParam.startTimer = false;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

    //Initiaze compare mode
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0
        );

    Timer_A_initCompareModeParam initCompParam = {0};
    initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    initCompParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    initCompParam.compareValue = 240*n;   // gives about a 80 uS delay
    Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

    Timer_A_startCounter( TIMER_A1_BASE,
            TIMER_A_CONTINUOUS_MODE
                );

    //Enter LPM0, enable interrupts -  wait for count to be reached
    __bis_SR_register(LPM0_bits + GIE);

    // wake up here when count reached
    Timer_A_stop(TIMER_A1_BASE);

    // set test gpio low
    //GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A1_VECTOR)))
#endif
void TIMER1_A1_ISR (void)
{
    switch (__even_in_range(TA1IV,6)){
    case 0:
        break;  //TA1IV_NONE
    case 2:
        // interrupt happends when count value is reached - so wake up the system
        __bic_SR_register_on_exit(LPM0_bits); // wakeup
        break;  // TA1IV_TACCR1
    case 4:
        break; // TA1IV_TACCR2
    }

}


