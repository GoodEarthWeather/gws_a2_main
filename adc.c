/*
 * adc.c
 *
 *  Created on: May 11, 2018
 *      Author: dmcneill
 */
#include "main.h"


uint16_t secondaryBatteryResult;
uint16_t vbatOKResult;

// test comment

void measureADC(void) // this will measure only the vbatt and vsolar system voltages
{
    extern struct sysResults wxOut;
    extern struct sysResultsTime wxOutTime;
    extern uint32_t epochTime;

    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN1); // set high to enable vbatt voltage to connect to divider
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); // set high to enable vsolar voltage to connect to divider
    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;  // wait until reference is ready
    Ref_A_enableReferenceVoltage(REF_A_BASE);  // turn on reference voltage
    __delay_cycles(600);  // wait 75uS for reference to settle

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    // Configure interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG3);
    //Enable memory buffer 3 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,ADC12_B_IE3,0,0);

    //Enable/Start first sampling and conversion cycle
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_START_AT_ADC12MEM2,   // start measurement at memory2
                            ADC12_B_SEQOFCHANNELS);

    // wait until ADC is done - busy loop
    //while (ADC12_B_isBusy(ADC12_B_BASE) == ADC12_B_BUSY) ;
    // wait until ADC is done - go to sleep and let interrupt wake up
    __bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_B_ISR will force exit

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN1); // set low to disable vbatt system voltage to connect to divider
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); // set low to disable vsolar system voltage to connect to divider
    // now get results
    wxOut.primaryBattery = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_2);  // primaryBattery = VBATT
    wxOut.secondaryBattery = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_3); // secondaryBattery = VSOLAR
    wxOutTime.battery = epochTime;
    ADC12_B_disable(ADC12_B_BASE);
    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;  // wait until reference is ready
    Ref_A_disableReferenceVoltage(REF_A_BASE);  // turn off reference voltage

}

void measureWindDirection(struct windDirection *wd)
{

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // set high to enable shunt regulator for wind direction measurement
    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;  // wait until reference is ready
    Ref_A_enableReferenceVoltage(REF_A_BASE);  // turn on reference voltage
    __delay_cycles(600);  // wait 75uS for reference to settle

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    // Configure interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);
    //Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,ADC12_B_IE1,0,0);

    //Enable/Start first sampling and conversion cycle
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_START_AT_ADC12MEM0,
                            ADC12_B_SEQOFCHANNELS);


    // wait until ADC is done - busy loop
    //while (ADC12_B_isBusy(ADC12_B_BASE) == ADC12_B_BUSY) ;
    // wait until ADC is done - go to sleep and let interrupt wake up
    __bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_B_ISR will force exit
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // turn off shunt regulator

    // now get results
    wd->directionCode = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);
    wd->referenceCode = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1);
    ADC12_B_disable(ADC12_B_BASE);

    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;  // wait until reference is ready
    Ref_A_disableReferenceVoltage(REF_A_BASE);  // turn off reference voltage


}


void initADC(void)
{
    extern struct sysResults wxOut;
    extern struct sysResultsTime wxOutTime;
    extern uint32_t epochTime;

    // Initialize the ADC
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // set resolution
    //ADC12_B_setResolution(ADC12_B_BASE, ADC12_B_RESOLUTION_10BIT);

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    /*
    * Base address of ADC12B Module
    * For memory buffers 0-7 sample/hold for 16 clock cycles
    * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
    * Disable Multiple Sampling
    */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
      ADC12_B_CYCLEHOLD_32_CYCLES,
      ADC12_B_CYCLEHOLD_4_CYCLES,
      ADC12_B_MULTIPLESAMPLESENABLE);

    //Configure Memory Buffer
    // MEMORY_0 = A12 = P3.0 = Wind Direction Input
    // MEMORY_1 = A0 = P1.0 = Wind Direction Reference
    // MEMORY_2 = A14 = P3.2 = VBATT system voltage input
    // MEMORY_3 = A4 = P1.4 = VSOLAR system voltage input

    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A12;
    //configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_EXTPOS_VREFNEG_VSS;  // external voltage reference
    //configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS; // internal VCC voltage reference
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS; // internal Vref voltage reference
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    // configure memory 0
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);
    // configure memory 1
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A0;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);
    // configure memory 2
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_2;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A14;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);
    // configure memory 3
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_3;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A4;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    ADC12_B_disable(ADC12_B_BASE);

    // now set up voltage reference
    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;  // wait until reference is ready
    Ref_A_setReferenceVoltage(REF_A_BASE,REF_A_VREF1_2V);  // select 1.2V reference
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif
void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV,20))
    {
    case  0: break;                         // Vector  0:  No interrupt
    case  2: break;                         // Vector  2:  ADC12BMEMx Overflow
    case  4: break;                         // Vector  4:  Conversion time overflow
    case  6: break;                         // Vector  6:  ADC12BHI
    case  8: break;                         // Vector  8:  ADC12BLO
    case 10: break;                         // Vector 10:  ADC12BIN
    case 12: break;                         // Vector 12:  ADC12BMEM0 Interrupt
    case 14:
        // conversion completed for MEM1 - can now exit
        ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);
        ADC12_B_disableInterrupt(ADC12_B_BASE,ADC12_B_IE1,0,0);
        __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
        break;                         // Vector 14:  ADC12BMEM1
    case 16: break;                         // Vector 16:  ADC12BMEM2
    case 18:
        // conversion completed for MEM3 - can now exit
        ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG3);
        ADC12_B_disableInterrupt(ADC12_B_BASE,ADC12_B_IE3,0,0);
        __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
        break;                         // Vector 18:  ADC12BMEM3
    case 20: break;                         // Vector 20:  ADC12BMEM4
    case 22: break;                         // Vector 22:  ADC12BMEM5
    case 24: break;                         // Vector 24:  ADC12BMEM6
    case 26: break;                         // Vector 26:  ADC12BMEM7
    case 28: break;                         // Vector 28:  ADC12BMEM8
    case 30: break;                         // Vector 30:  ADC12BMEM9
    case 32: break;                         // Vector 32:  ADC12BMEM10
    case 34: break;                         // Vector 34:  ADC12BMEM11
    case 36: break;                         // Vector 36:  ADC12BMEM12
    case 38: break;                         // Vector 38:  ADC12BMEM13
    case 40: break;                         // Vector 40:  ADC12BMEM14
    case 42: break;                         // Vector 42:  ADC12BMEM15
    case 44: break;                         // Vector 44:  ADC12BMEM16
    case 46: break;                         // Vector 46:  ADC12BMEM17
    case 48: break;                         // Vector 48:  ADC12BMEM18
    case 50: break;                         // Vector 50:  ADC12BMEM19
    case 52: break;                         // Vector 52:  ADC12BMEM20
    case 54: break;                         // Vector 54:  ADC12BMEM21
    case 56: break;                         // Vector 56:  ADC12BMEM22
    case 58: break;                         // Vector 58:  ADC12BMEM23
    case 60: break;                         // Vector 60:  ADC12BMEM24
    case 62: break;                         // Vector 62:  ADC12BMEM25
    case 64: break;                         // Vector 64:  ADC12BMEM26
    case 66: break;                         // Vector 66:  ADC12BMEM27
    case 68: break;                         // Vector 68:  ADC12BMEM28
    case 70: break;                         // Vector 70:  ADC12BMEM29
    case 72: break;                         // Vector 72:  ADC12BMEM30
    case 74: break;                         // Vector 74:  ADC12BMEM31
    case 76: break;                         // Vector 76:  ADC12BRDY
    default: break;
    }
}

