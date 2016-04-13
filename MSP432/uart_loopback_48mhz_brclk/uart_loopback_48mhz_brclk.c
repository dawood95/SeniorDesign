/*
 * -------------------------------------------
 *    MSP432 DriverLib - v2_20_00_08 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 UART - Loopback with 48MHz DCO BRCLK
 *
 * Description: This demo connects TX to RX of the MSP432 UART
 * The example code shows proper initialization of registers
 * and interrupts to receive and transmit data. If data is incorrect P1.0 LED
 * is turned ON.
 *
 *  MCLK = HSMCLK = SMCLK = DCO of 48MHz
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *       RST -|     P1.3/UCA0TXD|----|
 *            |                 |    |
 *           -|                 |    |
 *            |     P1.2/UCA0RXD|----|
 *            |                 |
 *            |             P1.0|---> LED
 *            |                 |
 *
 * Author: Timothy Logan
*******************************************************************************/

/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include "printf.h"
#include <stdbool.h>
#include "uart_test.h"

/* Header Includes */
//#define START_BYTE 0xFF
//#define END_BYTE 0x00
//#define NO_SUPPORTED_INSTR 0x0a
#define MAX_FRAME_LENGTH 10
//#define CELL_NUM_MASK 0x0000000F
//#define NUM_VOLTAGE_BITS_BYTE2 8
//#define VOLTAGE_MASK 0x000000F0
//#define MAX_CELL_VOLTAGE 4.2
//#define MAX_VOLTAGE_DATA 2100
#define JOY_BUFF_SIZE 7
#define TELEMETRY_BUFF_SIZE 4

/* Static Definitions */
uint8_t joy_buff[JOY_BUFF_SIZE];
volatile int joy_idx = 0;
char telemetry_buff[TELEMETRY_BUFF_SIZE];
volatile int telemetry_idx = 0;
float vbat;
int vbat_flag;

/*************************** Serial Debugging (through USB-UART) Setup ***************************/
const eUSCI_UART_Config debugLoggingConfig =
{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                      // BRDIV = 26
        2,                                       // UCxBRF = 0
        0,                                       // UCxBRS = 111
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    78,                                      // BRDIV = 78
    2,                                       // UCxBRF = 2
    0,                                       // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

const eUSCI_UART_Config vbatUartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    78,                                      // BRDIV = 78
    2,                                       // UCxBRF = 2
    0,                                       // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

/****************************** Timer_A PWM Configuration Parameter ******************************/
/* AETR Channels */
Timer_A_PWMConfig pwmConfigA =
{
        TIMER_A_CLOCKSOURCE_ACLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        128,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
		TIMER_A_OUTPUTMODE_TOGGLE_SET,
        64
};

Timer_A_PWMConfig pwmConfigB = {
		TIMER_A_CLOCKSOURCE_ACLK,
		TIMER_A_CLOCKSOURCE_DIVIDER_1,
		128,
		TIMER_A_CAPTURECOMPARE_REGISTER_3,
		TIMER_A_OUTPUTMODE_TOGGLE_SET,
		64
};

Timer_A_PWMConfig pwmConfigC = {
		TIMER_A_CLOCKSOURCE_ACLK,
		TIMER_A_CLOCKSOURCE_DIVIDER_1,
		128,
		TIMER_A_CAPTURECOMPARE_REGISTER_2,
		TIMER_A_OUTPUTMODE_TOGGLE_SET,
		64
};

Timer_A_PWMConfig pwmConfigD = {
		TIMER_A_CLOCKSOURCE_ACLK,
		TIMER_A_CLOCKSOURCE_DIVIDER_1,
		128,
		TIMER_A_CAPTURECOMPARE_REGISTER_1,
		TIMER_A_OUTPUTMODE_TOGGLE_SET,
		64
};

/* AUX Channels */
Timer_A_PWMConfig pwmConfigE = {
		TIMER_A_CLOCKSOURCE_ACLK,
		TIMER_A_CLOCKSOURCE_DIVIDER_1,
		128,
		TIMER_A_CAPTURECOMPARE_REGISTER_1,
		TIMER_A_OUTPUTMODE_TOGGLE_SET,
		80
};

/* Ultrasonic Sensor Configuration */
const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 3MHz
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
};

/* Timer_A Capture Mode Configuration Parameter */
const Timer_A_CaptureModeConfig captureModeConfig_CCR1 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,        // CC Register 2
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

const Timer_A_CaptureModeConfig captureModeConfig_CCR2 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,        // CC Register 2
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

void printFrame() {
	/*int i = 0;
	int voltage_raw = ((telemetry_buff[2] & CELL_NUM_MASK) << NUM_VOLTAGE_BITS_BYTE2) + telemetry_buff[3];
    int cell_num = (telemetry_buff[2] & VOLTAGE_MASK) >> 4;
    float voltage = voltage_raw;
    voltage = (float)((float)(voltage/MAX_VOLTAGE_DATA) * MAX_CELL_VOLTAGE);

	printf(EUSCI_A0_MODULE, "Cell Num : %i Voltage Value:  %i\r\n", cell_num, (int)voltage);
	printf(EUSCI_A0_MODULE, "Total Battrey Voltage : %i\r\n", (int)(voltage*4));
	for (i = 0; i < 4; i++) {
		printf(EUSCI_A0_MODULE, "Received Byte:  %x\r\n", telemetry_buff[i]);
	}*/
	int cell_num = telemetry_buff[(telemetry_idx + 3) % 4] >> 4;
	int voltage_raw = ((telemetry_buff[(telemetry_idx + 3) % 4] & 0x0F) << 8) + telemetry_buff[telemetry_idx];

	float voltage = (voltage_raw / 2100.0) * 4.2;
	printf(EUSCI_A0_MODULE, "Cell %i is at %iV (received %x %x %x %x)\r\n", cell_num, (int) voltage,
			telemetry_buff[(telemetry_idx + 1) % 4], telemetry_buff[(telemetry_idx + 2) % 4], telemetry_buff[(telemetry_idx + 3) % 4], telemetry_buff[telemetry_idx]);
	//printf(EUSCI_A0_MODULE, "%x\r\n", telemetry_buff[telemetry_idx]);
	//printf(EUSCI_A0_MODULE, "pitch: %i\r\n", telemetry_buff[(byte_cnt + 2) % 6]);
	//printf(EUSCI_A0_MODULE, "roll: %i\r\n", telemetry_buff[(byte_cnt + 3) % 6]);
	//if (telemetry_buff[(byte_cnt + 4) % 6] < 64 )
	//	printf(EUSCI_A0_MODULE, "yaw: %i\r\n", telemetry_buff[(byte_cnt + 4) % 6]);
	//printf(EUSCI_A0_MODULE, "throttle: %i\r\n\n", telemetry_buff[(byte_cnt + 5) % 6]);
	//int voltage_raw = ((telemetry_buff[2] & CELL_NUM_MASK) << NUM_VOLTAGE_BITS_BYTE2) + telemetry_buff[3];
}
void setupSerialLogging(void)
{

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_MODULE, &debugLoggingConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_MODULE);

    /* Setting DCO to 12MHz */
    //CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    MAP_Interrupt_enableMaster();
}

int main(void)
{
	/* Halting WDT  */
    MAP_WDT_A_holdTimer();

    /*Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
    		GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /*Uart Pin 2.2 for Battery Monitoring*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
    		GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting DCO to 48MHz (upping Vcore) */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE0);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Enable printf() serial debugging */
    setupSerialLogging();

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_MODULE, &uartConfig);
    MAP_UART_initModule(EUSCI_A1_MODULE, &vbatUartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_MODULE);
    MAP_UART_enableModule(EUSCI_A1_MODULE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);

    MAP_UART_enableInterrupt(EUSCI_A1_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    // MAP_Interrupt_enableSleepOnIsrExit();

    /********************************* PWM *********************************/
    /* Setting ACLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    //MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

    /* Configuring GPIO2.4 as peripheral output for PWM and P1.1 for button
     * interrupt */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    /*MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);*/

    /* Configuring GPIO2.5 as peripheral output for PWM */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring GPIO2.6 as peripheral output for PWM  and P1.4 for button
     * interrupt */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
    /*MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);*/

    /* Configuring GPIO2.7 as peripheral output for PWM */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,
           GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring GPIO7.7 as peripheral output for PWM */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7,
           GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    MAP_Timer_A_generatePWM(TIMER_A0_MODULE, &pwmConfigA);
    MAP_Timer_A_generatePWM(TIMER_A0_MODULE, &pwmConfigB);
    MAP_Timer_A_generatePWM(TIMER_A0_MODULE, &pwmConfigC);
    MAP_Timer_A_generatePWM(TIMER_A0_MODULE, &pwmConfigD);
    MAP_Timer_A_generatePWM(TIMER_A1_MODULE, &pwmConfigE);

    /* Enabling interrupts and starting the watchdog timer */
    //MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    printf(EUSCI_A0_MODULE, "Init done\r\n");
    while(1)
    {
        //MAP_UART_transmitData(EUSCI_A2_MODULE, TXData);
        //MAP_Interrupt_enableSleepOnIsrExit();
        //MAP_PCM_gotoLPM0InterruptSafe();
    }
}

/* EUSCI A0 UART ISR - Echos data back to PC host */
void euscia0_isr(void)
{
	volatile uint8_t RXData = 0;

	/* UART_TX Low Level RX Funtionality */
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_MODULE);
    MAP_UART_clearInterruptFlag(EUSCI_A2_MODULE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
    	RXData = MAP_UART_receiveData(EUSCI_A2_MODULE);
    	joy_buff[joy_idx] = RXData;
    	if (joy_buff[(joy_idx + 1) % JOY_BUFF_SIZE] == 0x9e && joy_buff[joy_idx] == 0xe9) {
    		pwmConfigA.dutyCycle = joy_buff[(joy_idx + 2) % JOY_BUFF_SIZE];
    		MAP_Timer_A_setCompareValue(TIMER_A0_MODULE, pwmConfigA.compareRegister, pwmConfigA.dutyCycle);

    		pwmConfigB.dutyCycle = joy_buff[(joy_idx + 5) % JOY_BUFF_SIZE];
    		MAP_Timer_A_setCompareValue(TIMER_A0_MODULE, pwmConfigB.compareRegister, pwmConfigB.dutyCycle);

    		pwmConfigC.dutyCycle = joy_buff[(joy_idx + 3) % JOY_BUFF_SIZE];
    		MAP_Timer_A_setCompareValue(TIMER_A0_MODULE, pwmConfigC.compareRegister, pwmConfigC.dutyCycle);

    		pwmConfigD.dutyCycle = joy_buff[(joy_idx + 4) % JOY_BUFF_SIZE];
    		MAP_Timer_A_setCompareValue(TIMER_A0_MODULE, pwmConfigD.compareRegister, pwmConfigD.dutyCycle);

    		pwmConfigE.dutyCycle = 80 + 32*(joy_buff[(joy_idx + 6) % JOY_BUFF_SIZE]);
    		MAP_Timer_A_setCompareValue(TIMER_A1_MODULE, pwmConfigE.compareRegister, pwmConfigE.dutyCycle);
    	}
    	joy_idx = (joy_idx + 1) % JOY_BUFF_SIZE;
    	    	/*RXData = MAP_UART_receiveData(EUSCI_A2_MODULE);
    	    	telemetry_buff[telemetry_idx] = RXData;

    	    	if ((telemetry_buff[(telemetry_idx + 1) % TELEMETRY_BUFF_SIZE] == 0x5e) && (telemetry_buff[(telemetry_idx + 2) % TELEMETRY_BUFF_SIZE] == 0x6)) {
    	        	//MAP_Interrupt_disableInterrupt(INT_EUSCIA1);
    	        	//vbat_flag = 1;
    	    		//MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[(telemetry_idx + 1) % TELEMETRY_BUFF_SIZE]);
    	    		//MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[(telemetry_idx + 3) % TELEMETRY_BUFF_SIZE]);
    	    		//MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[telemetry_idx]);
    	    		//MAP_UART_transmitData(EUSCI_A2_MODULE, (char) 0xe5);
    	    		//MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    	    		printFrame();
    	    	}
    	    	//printFrame();
    	    	telemetry_idx = (telemetry_idx + 1) % TELEMETRY_BUFF_SIZE;*/

    }
    //MAP_Interrupt_disableSleepOnIsrExit();
}

void euscia1_isr(void)
{
	volatile uint8_t RXData = 0;

	/* UART_TX Low Level RX Funtionality */
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_MODULE);
    MAP_UART_clearInterruptFlag(EUSCI_A1_MODULE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
    	RXData = MAP_UART_receiveData(EUSCI_A1_MODULE);
    	telemetry_buff[telemetry_idx] = RXData;
    	if (telemetry_buff[(telemetry_idx + 1) % TELEMETRY_BUFF_SIZE] == 0x5e && telemetry_buff[(telemetry_idx + 2) % TELEMETRY_BUFF_SIZE] == 0x6) {
        	MAP_Interrupt_disableInterrupt(INT_EUSCIA1);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[(telemetry_idx + 1) % TELEMETRY_BUFF_SIZE]);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[(telemetry_idx + 3) % TELEMETRY_BUFF_SIZE]);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[telemetry_idx]);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, (char) 0xe5);
    		MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    	}
    	telemetry_idx = (telemetry_idx + 1) % TELEMETRY_BUFF_SIZE;
    }
    //MAP_Interrupt_disableSleepOnIsrExit();
}

/* Port1 ISR - This ISR will progressively step up the duty cycle of the PWM
 * on a button press
 */
void port1_isr(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1) {
    	if(pwmConfigA.dutyCycle == 128)
    		pwmConfigA.dutyCycle = 64;
    	else
    		pwmConfigA.dutyCycle += 8;

    	MAP_Timer_A_generatePWM(TIMER_A0_MODULE, &pwmConfigA);
    } else if (status & GPIO_PIN4) {
    	if (pwmConfigC.dutyCycle == 128)
    		pwmConfigC.dutyCycle = 64;
    	else
    		pwmConfigC.dutyCycle += 8;
    	MAP_Timer_A_generatePWM(TIMER_A0_MODULE, &pwmConfigC);
    }
}

