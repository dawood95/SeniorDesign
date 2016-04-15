#include "driverlib.h"
#include "printf.h"
#include <msp.h>

#include <stdint.h>

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

/*const Timer_A_PWMConfig triggerConfig = {
    TIMER_A_CLOCKSOURCE_ACLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    0xFFFF/8,
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_OUTPUTMODE_SET_RESET,
    0xFFFF/8 - 2
};*/

const Timer_A_UpModeConfig upModeConfig = {
	TIMER_A_CLOCKSOURCE_ACLK,       		// ACLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_16,  		// ACLK/1 = 128kHz
	0xFFFF/4,			 						// Period = 1ms
	TIMER_A_TAIE_INTERRUPT_DISABLE,
	TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,	// Disable Timer ISR
    TIMER_A_SKIP_CLEAR                   	// Skup Clear Counter
};

const Timer_A_CaptureModeConfig echoConfig = {
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,
    TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
    TIMER_A_CAPTURE_SYNCHRONOUS,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
    TIMER_A_OUTPUTMODE_OUTBITVALUE
};

void setupSerialLogging(void)
{
    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &debugLoggingConfig);
    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);
    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    MAP_Interrupt_enableMaster();
}

/*float start = 0;
float distance = 0;
int warnCount = 0;
volatile int timerCount = 0;*/

int risingedge = 1;
int sensorflag = 1;
int pulsewidth = 0;


// Flags
int echo_neg_edge_rcvd = 1;

int overflow_count = 0;


void trigger_sensor(void);

int main(void)
{
    /* Stop watchdog timer */
    MAP_WDT_A_holdTimer();
    
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);

    /* Configuring P2.4 as output */
    //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    //MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);

    /* Starting and enabling ACLK (32kHz) */
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_4);
    
    MAP_Timer_A_configureUpMode(TIMER_A2_MODULE, &upModeConfig);
    MAP_Timer_A_startCounter(TIMER_A2_MODULE, TIMER_A_UP_MODE);
    /* Configuring Capture Mode */
    //MAP_Timer_A_initCapture(TIMER_A2_MODULE, &echoConfig);
    //MAP_Timer_A_generatePWM(TIMER_A2_BASE, &triggerConfig);
    
    /* Enabling interrupts and going to sleep */
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    MAP_Interrupt_enableInterrupt(INT_TA2_N);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    Timer_A_enableInterrupt(TIMER_A2_BASE);
    /* Enable printf() serial debugging */
    setupSerialLogging();
    
    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();

    printf(EUSCI_A2_BASE, "Init done\r\n");

    while(1) {
    	//MAP_PCM_gotoLPM0();
    }

}

//******************************************************************************
//
//This is the TIMERA interrupt vector service routine.
//
//******************************************************************************
void calculateEcho(void)
{
    /*uint16_t interruptValue = 0;

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
        if (distance < 40)
        	warnCount += 1;
        else
        	warnCount = 0;
        if(warnCount > 6)
        	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
        else
        	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);*/
	int temp;

	if (risingedge) {
		if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW) {
			//ignore
		} else {
			risingedge = 0;
			pulsewidth = Timer_A_getCaptureCompareCount(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
		}
	} else {
		if (Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT) == TIMER_A_CAPTURECOMPARE_INPUT_HIGH) {
			//ignore
		} else {
			risingedge = 1;
			echo_neg_edge_rcvd = 1;
			temp = Timer_A_getCaptureCompareCount(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
			pulsewidth = (temp - pulsewidth <= 0) ? (0x3333 - pulsewidth) + overflow_count*0x3333 : temp - pulsewidth;
			if ((pulsewidth*7.8125)/58 < 30) {
				MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
			} else {
				MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
			}
			overflow_count = 0;
		}
	}
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
}

void countOverflowSinceStart(void) {
	overflow_count++;
	//if (echo_neg_edge_rcvd) {
	//	echo_neg_edge_rcvd = 0;
		trigger_sensor();
	//}
}

void trigger_sensor() {
	volatile uint32_t i;

	MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
	for (i = 0 ; i < 2; i++);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6);
	for (i = 0; i < 2; i++);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
}
