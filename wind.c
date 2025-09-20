/*
 * wind.c
 *
 *  Created on: May 10, 2018
 *      Author: dmcneill
 */
#include "main.h"

// Define wind speed variables for computing gust values
uint16_t currentWindValue = 0;  // current wind counter value
uint16_t currentGustValue = 0;  // current gust counter value
uint32_t currentGustTime;

uint16_t minuteWindValue = 0; // accumulation of wind counts over 1 minute
struct windDirection windD;
struct windDirection currentGustDirection;


// initialize wind speed timer
void initWind(void)
{
    //Start timer in up mode
    Timer_A_initUpModeParam initUpParam = {0};
    initUpParam.clockSource = TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK;
    initUpParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initUpParam.timerPeriod = 0;
    initUpParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initUpParam.captureCompareInterruptEnable_CCR0_CCIE =
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initUpParam.timerClear = TIMER_A_DO_CLEAR;
    initUpParam.startTimer = false;
    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam);

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);

}

// This function is called on the 1 minute interval and will
// copy the wind and gust values to the wx structure.
void recordWind(void)
{
    extern struct sysResults wxOut;
    extern struct sysResultsTime wxOutTime;
    extern uint32_t epochTime;

    // record values before clearing
    wxOut.windCount = minuteWindValue; // accumulated wind counts over 1 minute (summation of 12 5 second counts)
    wxOut.gustCount = currentGustValue;
    wxOut.gustDirectionCode = currentGustDirection.directionCode;
    wxOut.gustDirectionReferenceCode = currentGustDirection.referenceCode;
    measureWindDirection(&windD);
    wxOut.windDirectionCode = windD.directionCode;
    wxOut.windDirectionReferenceCode = windD.referenceCode;
    wxOutTime.windDirection = epochTime;
    wxOutTime.windCount = epochTime;
    wxOutTime.gustCount = currentGustTime;

    // now clear out variables
    minuteWindValue = 0;
    currentGustValue = 0;
}
