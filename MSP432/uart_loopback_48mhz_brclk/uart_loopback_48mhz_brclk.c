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

//Ultranic Sensors
#define NUM_TIMER_CAPTURES       20
#define NUM_ULTRA_SENSORS        3  //4
#define TIMER_COUNTER_PERIOD     0xFFF0
#define	TIMER_TRIGGER_COMPARE_VALUE 0xFFEA
#define MAX_COUNT                 0xFFFF
#define TRIGGER_PULSE_WIDTH       10000
#define TRIGGER_PULSE_DELAY       500
#define TIMCCTL_ENBIT             0x0010
#define TIMCCTL_INTBIT            0x0001
#define SAFE_ECHOWIDTH_COUNT 20000
#define ALERT_ECHOWIDTH_COUNT 10000
#define SAFE_ECHO_DISTANCE 100

/* Static Definitions */
uint8_t joy_buff[JOY_BUFF_SIZE];
volatile int joy_idx = 0;
uint8_t telemetry_buff[TELEMETRY_BUFF_SIZE];
volatile int telemetry_idx = 0;
float vbat;
int vbat_flag;

/* Ultrasonic Sensors */
static volatile uint32_t timerAcaptureValues[NUM_TIMER_CAPTURES];
static volatile uint32_t recentDirDistances[NUM_ULTRA_SENSORS];
static volatile uint32_t timerAcapturePointer = 0;
static volatile uint32_t risingEdgeCnt[NUM_ULTRA_SENSORS], fallEdgeCnt[NUM_ULTRA_SENSORS], timerFlag[NUM_ULTRA_SENSORS];
float start = 0;
float distance = 0;
int warnCount[NUM_ULTRA_SENSORS];
volatile int timerCount = 0;


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

/********************* Ultrasonic Sensor Configuration *************************************/
// Utilizing Up Mode configuration for Timer Modules
const Timer_A_PWMConfig triggerConfig = {
    TIMER_A_CLOCKSOURCE_ACLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    0xFFFF/8,
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_OUTPUTMODE_SET_RESET,
    0xFFFF/8 - 2
};

const Timer_A_CaptureModeConfig echoConfig = {
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,
    TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
    TIMER_A_CAPTURE_SYNCHRONOUS,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
    TIMER_A_OUTPUTMODE_OUTBITVALUE
};

/* Timer Module A2 */
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 128kHz
		TIMER_COUNTER_PERIOD,			 // Period = 1ms
		TIMER_A_TAIE_INTERRUPT_DISABLE,
		TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,  // Disable Timer ISR
        TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
};

/* Configurations for Trigger Signals*/
const Timer_A_CompareModeConfig compareModeConfig_CCR2 =
{
		TIMER_A_CAPTURECOMPARE_REGISTER_2,        // CC Register 2
		TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,  // Enable interrupt
		TIMER_A_OUTPUTMODE_TOGGLE_RESET,
		TIMER_TRIGGER_COMPARE_VALUE
};

const Timer_A_CompareModeConfig compareModeConfig_CCR4 =
{
		TIMER_A_CAPTURECOMPARE_REGISTER_4,        // CC Register 4
		TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,  // Enable interrupt
		TIMER_A_OUTPUTMODE_TOGGLE_RESET,
		TIMER_TRIGGER_COMPARE_VALUE
};

/* Timer_A Capture Mode Configurations */
const Timer_A_CaptureModeConfig captureModeConfig_CCR1 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,        // CC Register 2
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

const Timer_A_CaptureModeConfig captureModeConfig_CCR3 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,        // CC Register 3
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

const Timer_A_CaptureModeConfig captureModeConfig_CCR4 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_4,        // CC Register 3
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};


void sendTrigger(void);

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

int main(void){
    int i = 0;
	/* Halting WDT  */

    MAP_WDT_A_holdTimer();

    /*Selecting P3.2 and P3.3 in UART mode */
   // MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
    //		GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /*Uart Pin 2.2 for Battery Monitoring*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
    		GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting DCO to 48MHz (upping Vcore) */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE0);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    for (i = 0; i < 3; i++) {
    	timerFlag[i] = 0;
    }
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

    /****************** Ultrasonic Timer Modules ******/

    /* Configuring P5.6, P6.6  as echo input pins*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN6,
                GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring P5.7, P6.7  as trigger output pins*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7,
                GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN7,
                GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8, GPIO_PIN2,
                GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9, GPIO_PIN2,
                GPIO_PRIMARY_MODULE_FUNCTION);


    /*Testing with LED *//*
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
*/
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN6);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

    /* Intializing TA2 Module */
    MAP_Timer_A_configureUpMode(TIMER_A2_MODULE, &upModeConfig);

    MAP_Timer_A_startCounter(TIMER_A2_MODULE, TIMER_A_UP_MODE);

    MAP_Timer_A_initCapture(TIMER_A2_MODULE, &captureModeConfig_CCR1);
    MAP_Timer_A_initCapture(TIMER_A2_MODULE, &echoConfig);

    //MAP_Timer_A_initCompare(TIMER_A2_MODULE, &triggerConfig);
    MAP_Timer_A_initCompare(TIMER_A2_MODULE, &compareModeConfig_CCR4);

    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
  //  MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_3);


    MAP_Interrupt_enableInterrupt(INT_TA2_N);

    MAP_Timer_A_generatePWM(TIMER_A2_MODULE, &echoConfig);
    /* initializing TA3 Module */

    MAP_Timer_A_configureUpMode(TIMER_A3_MODULE, &upModeConfig);
    MAP_Timer_A_startCounter(TIMER_A3_MODULE, TIMER_A_UP_MODE);

    MAP_Timer_A_initCapture(TIMER_A3_MODULE, &captureModeConfig_CCR3);
    MAP_Timer_A_initCompare(TIMER_A3_MODULE, &compareModeConfig_CCR2);

    //MAP_Timer_A_generatePWM(TIMER_A3_, &triggerConfig);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_3);

    MAP_Interrupt_enableInterrupt(INT_TA3_N);
    /* Enabling interrupts and starting the watchdog timer */
    //MAP_Interrupt_enableInterrupt(INT_PORT1);

    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    /***************Warning Data***************/
    warnCount[0] = 0;

    printf(EUSCI_A0_MODULE, "Init done\r\n");
    sendTrigger();
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
	int cell_num;
	int raw_voltage;
	float voltage;
	uint8_t int_voltage, frac_voltage;

	/* UART_TX Low Level RX Funtionality */
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_MODULE);
    MAP_UART_clearInterruptFlag(EUSCI_A1_MODULE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
    	RXData = MAP_UART_receiveData(EUSCI_A1_MODULE);
    	telemetry_buff[telemetry_idx] = RXData;
    	if (telemetry_buff[(telemetry_idx + 1) % TELEMETRY_BUFF_SIZE] == 0x5e && telemetry_buff[(telemetry_idx + 2) % TELEMETRY_BUFF_SIZE] == 0x6) {
        	MAP_Interrupt_disableInterrupt(INT_EUSCIA1);
    		/*MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[(telemetry_idx + 1) % TELEMETRY_BUFF_SIZE]);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[(telemetry_idx + 3) % TELEMETRY_BUFF_SIZE]);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, telemetry_buff[telemetry_idx]);
    		MAP_UART_transmitData(EUSCI_A2_MODULE, (char) 0xe5);*/
        	cell_num = telemetry_buff[(telemetry_idx + 3) % 4] >> 4;
        	raw_voltage = ((telemetry_buff[(telemetry_idx + 3) % 4] & 0x0F) << 8) + telemetry_buff[telemetry_idx];
        	voltage = (raw_voltage / 2100.0) * 4.2;
        	int_voltage = ((int) voltage) > 3 ? 1 : 0;
        	frac_voltage = ((int) ((voltage - ((int) voltage)) * 32));
        	MAP_UART_transmitData(EUSCI_A2_MODULE, ((cell_num << 6) | (int_voltage << 5) | (frac_voltage)));
        	//MAP_UART_transmitData(EUSCI_A2_MODULE, cell_num << 6);
        	//printf(EUSCI_A0_MODULE, "Cell %i is at %iV (received %x %x %x %x)\r\n", cell_num, (int) voltage,
        	//		telemetry_buff[(telemetry_idx + 1) % 4], telemetry_buff[(telemetry_idx + 2) % 4], telemetry_buff[(telemetry_idx + 3) % 4], telemetry_buff[telemetry_idx]);
    		MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    	}
    	telemetry_idx = (telemetry_idx + 1) % TELEMETRY_BUFF_SIZE;
    }
    //MAP_Interrupt_disableSleepOnIsrExit();
}

/*
 * Timer A2 ISR fro Ultrasonic 1
 *
 */
/*void timer_a2_n_isr(void)
{
    uint16_t  temp;
    uint16_t diffTime;
    int8_t sensor = -1;
    uint8_t port, pin;
    uint16_t currCnt;
    int16_t rising_val = -1, falling_val = -1;
    uint8_t sensor_flag = 0;

   if((Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURE_OVERFLOW) != 0){
       //printf(EUSCI_A2_BASE, "overflow\r\n");
	   TA2CCTL1 &= ~COV;
       MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
       return;
   }

   /*if((Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3) & TIMER_A_CAPTURE_OVERFLOW) != 0){
       //printf(EUSCI_A2_BASE, "overflow\r\n");
	   TA2CCTL3 &= ~COV;
       MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
       return;
   }*/


 /*  if((TA2CCTL1 & TIMCCTL_ENBIT) && (TA2CCTL1 & TIMCCTL_INTBIT))
   {
	   //MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A2_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_3);
	   sensor = 0;
	   sensor_flag = 1;
       currCnt = MAP_Timer_A_getCaptureCompareCount(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
   	   if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_HIGH) {
   	           //printf(EUSCI_A2_BASE, "start : %i\r\n",interruptValue);
   	       risingEdgeCnt[sensor] = currCnt;
   		   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
   	   }
   	   else{
   	   //if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW) {
   		   //fallEdgeCnt[sensor] = currCnt;
   		   //if (fallEdgeCnt[sensor] < risingEdgeCnt[sensor]) {
   		//   diffTime = (TIMER_COUNTER_PERIOD - risingEdgeCnt[sensor]) + fallEdgeCnt[sensor];
   			 //  temp = diffTime;
   		 //  } else {
   	//		   diffTime = fallEdgeCnt[sensor] - risingEdgeCnt[sensor];
   	//		   temp = diffTime;
   	//	   }
   	 //      if(diffTime < 1800)
   	    	   MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
   	  //     else
   	   // 	   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
   	   }


       MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }

    /*if((TA2CCTL3 & TIMCCTL_ENBIT) && (TA2CCTL3 & TIMCCTL_INTBIT))
     {
    	MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A2_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
       	sensor = 1;
       	sensor_flag = 1;
       	//MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
        currCnt = Timer_A_getCaptureCompareCount(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
        if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_HIGH) {
                //printf(EUSCI_A2_BASE, "start : %i\r\n",interruptValue);
                risingEdgeCnt[sensor] = currCnt;
        } else {
    		   fallEdgeCnt[sensor] = currCnt;
    		   if (fallEdgeCnt[sensor] < risingEdgeCnt[sensor]) {
    			   diffTime = (TIMER_COUNTER_PERIOD - risingEdgeCnt[sensor]) + fallEdgeCnt[sensor];
    			   temp = diffTime;
    		   } else {
    			   diffTime = fallEdgeCnt[sensor] - risingEdgeCnt[sensor];
    			   temp = diffTime;
    		   }
        }
        if(diffTime < 50)
    		MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
        else
        	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);

        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
     }
*/
    /* Config for Echo Status
     * timerFlag = 1 when echo signal is high
     * timerFlag = 0 whgen echo signal is low
     */
  		/*if(sensor == 0)
    		{	pin = GPIO_PIN5; port = GPIO_PORT_P4;}
    		else if(sensor == 1)
    		{   pin = GPIO_PIN6; port = GPIO_PORT_P4;}
*/

//    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
//    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_3);

    /* Processing existing nearest obtscle distance to evaluate next course of action */
    //evaluateQuadCopterSafety();
//}

/* Ultraosnic ISR for TA3 (Ultra2)
 *
 */

void countOverflowSinceStart(void){
	timerCount += 1;
}

void timer_a2_n_isr(void)
{

	uint16_t interruptValue = 0;

	    if((Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURE_OVERFLOW) != 0){
	        //printf(EUSCI_A2_BASE, "overflow\r\n");
	    	TA2CCTL1 &= ~COV;
	        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
	        return;
	    }
	    if((Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) == 0){
	        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
	        return;
	    }
	    interruptValue = Timer_A_getCaptureCompareCount(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
	    if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_HIGH) {
	            //printf(EUSCI_A2_BASE, "start : %i\r\n",interruptValue);
	            start = interruptValue;
	            timerCount = 0;
	    }
	    else  {
	        //no overflow -- valid start data
	        //printf(EUSCI_A2_BASE, "end : %i\r\n",interruptValue);
	        interruptValue += timerCount*0xFFFF;
	        distance = (interruptValue - start)*7.8125/58;
	        if (distance < 50)
	        	warnCount[0] += 1;
	        else
	        	warnCount[0] = 0;
	        if(warnCount[0] > 5)
	        	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
	        else
	        	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
	    }
	    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

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

void sendTrigger(void) {
	uint32_t i;
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7);
	for(i=0 ; i<2; i++);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN7);
	for(i = 0; i < 100; i++);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7);
	//for(i = 0; i < 100; i++);
}

/*
void calculateEcho(void)
{
    uint16_t interruptValue = 0;

    if((Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURE_OVERFLOW) != 0){
        //printf(EUSCI_A2_BASE, "overflow\r\n");
    	TA2CCTL1 &= ~COV;
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        return;
    }
    if((Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) == 0){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        return;
    }
    interruptValue = Timer_A_getCaptureCompareCount(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_HIGH) {
            //printf(EUSCI_A2_BASE, "start : %i\r\n",interruptValue);
            start = interruptValue;
            timerCount = 0;
    }
    else  {
        //no overflow -- valid start data
        //printf(EUSCI_A2_BASE, "end : %i\r\n",interruptValue);
        interruptValue += timerCount*0xFFFF;
        distance = (interruptValue - start)*7.8125/58;
        if (distance < 50)
        	warnCount += 1;
        else
        	warnCount = 0;
        if(warnCount > 3)
        	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
        else
        	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
}
*/
