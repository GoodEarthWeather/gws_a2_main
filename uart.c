/*
 * uart.c
 *
 *  Created on: May 17, 2018
 *      Author: dmcneill
 */
#include "main.h"

uint8_t uartPresent = 0;  // default to no uart present

// initialize uart
void initUART(void)
{
    // Only initialize UART if USB connector is plugged in
    if ( GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH )
    {

        // P2.0, P2.1 - UART RXD and TXD
        GPIO_setAsPeripheralModuleFunctionInputPin(
                GPIO_PORT_P2,
                GPIO_PIN0 + GPIO_PIN1,
                GPIO_SECONDARY_MODULE_FUNCTION
        );

        uartPresent = 1;
        // Configure UART
        EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar = 26;  // configured for 19200 baud rate
        param.firstModReg = 0;
        param.secondModReg = 214;
        param.parity = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode = EUSCI_A_UART_MODE;
        param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

        if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param)) { return; }

        // enable uart channel
        EUSCI_A_UART_enable(EUSCI_A0_BASE);

        EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
        // Enable USCI_A0 RX interrupt
        EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    }
    else
    {
        // no uart, so set uart pins to outputs and force low
        GPIO_setAsOutputPin(
            GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1
            );
        GPIO_setOutputLowOnPin(
            GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1
            );
    }
}

//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void USCI_A0_ISR(void)
{
    extern volatile uint8_t commandBuffer[];
    extern volatile uint8_t validCommand;
    static uint8_t i = 0;
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
        commandBuffer[i++] = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
        if ( i > commandBuffer[0] )
        {
            i = 0;
            validCommand = 1;
            __bic_SR_register_on_exit(LPM0_bits); // wake up
        }
        break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    }
}

