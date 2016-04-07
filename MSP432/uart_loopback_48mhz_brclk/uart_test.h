/*
 * uart_test.h
 *
 *  Created on: Mar 24, 2016
 *      Author: EE63pc10-user
 */

#ifndef UART_TEST_H_
#define UART_TEST_H_

typedef struct Instr {
	uint8_t THROTTLE;
	uint8_t ROLL;
	uint8_t YAW;
	uint8_t PITCH;
	uint8_t STOP;
	uint8_t LAND;
	uint8_t BEEP;
}Instr;


#endif /* UART_TEST_H_ */
