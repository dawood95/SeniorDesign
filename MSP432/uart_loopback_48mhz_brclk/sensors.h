/*
 * sensors.h
 *
 *  Created on: Apr 7, 2016
 *      Author: Archit
 */

#ifndef SENSORS_H_
#define SENSORS_H_

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





#endif /* SENSORS_H_ */
