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
/*******************************************************************************
* MSP432 Timer_A - VLO Period Capture
*
* Description: Capture a number of periods of the VLO clock and store them in
* an array. When the set number of periods is captured the program is trapped
* and the LED on P1.0 is toggled. At this point halt the program execution read
* out the values using the debugger.
* ACLK = VLOCLK = 14kHz (typ.), MCLK = SMCLK = default DCO  = 3MHz
*
*                MSP432P401
*             ------------------
*         /|\|                  |
*          | |                  |
*          --|RST         P1.0  |---> P1.0 LED
*            |                  |
*            |                  |
*            |                  |
*            |                  |
* Author: Timothy Logan
*******************************************************************************/
/* DriverLib Includes */
#include "driverlib.h"
#include "printf.h"
/* Standard Includes */
#include <stdint.h>
#define NUMBER_TIMER_CAPTURES       20
#define NUMBER_ULTRA_SENSORS        2 // 6

/* Timer Module Specific Defs */

#define TIMER_COUNTER_PERIOD        333
#define MAX_COUNT 0x0FFFF
#define TRIGGER_PULSE_WIDTH       10000
#define TRIGGER_PULSE_DELAY       500
#define TIMCCTL_ENBIT             0x0010
#define TIMCCTL_INTBIT            0x0001

/* QCopter Header definitions */
typedef enum
{
	DIR_LEFT,
	DIR_RIGHT,
	DIR_FORWARD,
	DIR_BACK,
	DIR_UP,
	DIR_BOTTOM,
	DIR_NONE
}ULTRA_DIR;

typedef enum
{
	COMPLETELY_SAFE,
	PROXIMITY_ALERT,
	COLLISION_ALARM
}SAFETY_LEVEL;

#define SAFE_ECHOWIDTH_COUNT 20000
#define ALERT_ECHOWIDTH_COUNT 10000
#define SAFE_ECHO_DISTANCE 100 // Unit in cm

/* Timer_A Continuous Mode Configuration Parameter */

const eUSCI_UART_Config debugLoggingConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 26
        2,                                       // UCxBRF = 0
        0,                                       // UCxBRS = 111
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

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
       CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    MAP_Interrupt_enableMaster();
}


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

void sendTrigger(void);
void evaluateQuadCopterSafety();
/* Function for processing obstacle distance from pulse width data obtained */

uint32_t calculateDistance(uint32_t echo_width)
{
	return echo_width * TIMER_COUNTER_PERIOD * 340 / 2000000;
}


/* Static Flags for QuadCopter State*/
static volatile uint8_t collisionAlert;
static ULTRA_DIR collision_dir;


/* Statics for Calculation */
static volatile uint32_t timerAcaptureValues[NUMBER_TIMER_CAPTURES];
static volatile uint32_t recentDirDistances[NUMBER_ULTRA_SENSORS];
static volatile uint32_t timerAcapturePointer = 0;
static volatile uint32_t prevCnt[2], currCnt[2], timerFlag[2];
static volatile uint32_t temp;
static volatile SAFETY_LEVEL currSafety;

/* Function for procesing ultrasonic captured data about obstacle distance
 TODO - Critical function for evaluating safety status of quadcopter
  Factors to consider -
  	  Distance from all directions
  	  Current direction of motion (obtained from remote control instructions)
  	  Current velocity of copter (if possible obtained from flight controller)

  Functionality :
  	  Process data and determine current safety level
  	  Send appropriate information to remote controller
	  Take emergency actions to avoid collision
*/

void evaluateQuadCopterSafety()
{
	/* simple implemetation of safety assessment */
	int i;
	for(i = 0; i < NUMBER_ULTRA_SENSORS; i++)
	{
		if(recentDirDistances[i] < ALERT_ECHOWIDTH_COUNT)
			currSafety = PROXIMITY_ALERT;
		else
			currSafety = COMPLETELY_SAFE;
	}
}

int main(void)
{
	/*prevCnt = {0,0};
	currCnt = {0,0};*/
	timerFlag[0] = timerFlag[1] = 0;
	//uint8_t alarmLED = 0;
	/* Stop watchdog timer */
    MAP_WDT_A_holdTimer();

    /* Setup Serial Logging Support */
    setupSerialLogging();

    /*Configure P2.5 and  P2.4 as module input pins*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5,
                GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configure pins P2.3 and P5.7 as module output pins */
    /*MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN3,
                   GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7,
                   GPIO_PRIMARY_MODULE_FUNCTION);
	*/
    /* Configuring P1.0, P1.5, P2.3, P5.1 as output pins */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);

    /* Setting ACLK = VLO = 14kHz */
    MAP_CS_initClockSignal(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Configuring Capture Mode */

    MAP_Timer_A_initCapture(TIMER_A0_MODULE, &captureModeConfig_CCR1);
    MAP_Timer_A_initCapture(TIMER_A0_MODULE, &captureModeConfig_CCR2);

    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(TIMER_A0_MODULE, &continuousModeConfig);

    /* Enabling interrupts and going to sleep */
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_1);

    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
   // MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
   // MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_4);
    MAP_Interrupt_enableInterrupt(INT_TA0_N);
   // MAP_Interrupt_enableMaster();

    /* Starting the Timer_A0 in continuous mode */
    MAP_Timer_A_startCounter(TIMER_A0_MODULE, TIMER_A_CONTINUOUS_MODE);

    printf(EUSCI_A0_MODULE, "Testing printf\r\n");

    // Software Generation of Trigger Signal
    // TODO - Replace with Hardware Induced Trigger Interrupt
    while (1) {
    	sendTrigger();

    	for(temp = 0 ; temp < 15000; temp++);

    	}
    // MAP_PCM_gotoLPM0();
}

//******************************************************************************
//
//This is the TIMERA interrupt vector service routine.
//
//******************************************************************************
void timer_a_ccr_isr(void)
{
    uint32_t  temp,jj;
    uint32_t diffTime, debug_prev, debug_curr;
    char sensor = -1;
    uint8_t port, pin;

    //MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_2);

    if((TA0CCTL1 & TIMCCTL_ENBIT) && (TA0CCTL1 & TIMCCTL_INTBIT))
    {
    	//printf(EUSCI_A0_MODULE, "Detected Timer A0.1 interrupt at Capture Pin\r\n");
    	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    	sensor = 0;
    	currCnt[0] = MAP_Timer_A_getCaptureCompareCount(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }

    if((TA0CCTL2 & TIMCCTL_ENBIT) && (TA0CCTL2 & TIMCCTL_INTBIT))
    {
    	//printf(EUSCI_A0_MODULE, "Detected Timer A0.2 interrupt at Capture Pin\r\n");
    	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    	sensor = 1;
    	currCnt[1] = MAP_Timer_A_getCaptureCompareCount(TIMER_A0_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    }

   // printf(EUSCI_A0_MODULE, "Status of CCTL registers TA0.1 %x TA0.2 %x\r\n", TA0CCTL1, TA0CCTL2);
    //debug_prev = prevCnt;
    //debug_curr = currCnt;
    //printf(EUSCI_A0_MODULE, "CCR2 current value : %i\r\n", currCnt);
    if(sensor != -1)
    {
     if (currCnt[sensor] < prevCnt[sensor]) {
    	diffTime = (MAX_COUNT - prevCnt[sensor]) + currCnt[sensor];
    	temp = diffTime;
    } else {
    	diffTime = currCnt[sensor] - prevCnt[sensor];
    	temp = diffTime;
    }

    prevCnt[sensor] = currCnt[sensor];
    //if (timerFlag == 1) {
    //	sendTrigger();
    //}
    if (timerFlag[sensor] == 0) {
    	timerFlag[sensor] = 1;
    	//diffTime = 0;
    } else {
    	timerFlag[sensor] = 0;

    	if(sensor == 0)
    	{	pin = GPIO_PIN1; port = GPIO_PORT_P5;}
    	else
    	{   pin = GPIO_PIN3; port = GPIO_PORT_P2;}

    	if(diffTime < 15000)
    		GPIO_setOutputHighOnPin(port, pin);
    	else
    		GPIO_setOutputLowOnPin(port, pin);

    	recentDirDistances[sensor] = diffTime;

    	/* Processing existing nearest obtscle distance to evaluate next course of action */
    	evaluateQuadCopterSafety();

    	//printf(EUSCI_A0_MODULE, "Obtained Echo count : %u\r\n Calcuated distance = %n\r\n", diffTime, calculateDistance(diffTime));
    	//printf(EUSCI_A0_MODULE, "Obtained Echo count : %u\r\n", diffTime);
    	//for(jj = 0; jj <= 100; jj++)
       // sendTrigger();
       // sendTrigger();

    }
    }
}

void sendTrigger(void) {
	uint32_t i;
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
	for(i=0;i<TRIGGER_PULSE_DELAY;i++);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
	for(i = 0; i < TRIGGER_PULSE_WIDTH; i++);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
	for(i = 0; i < 10000; i++);
}
