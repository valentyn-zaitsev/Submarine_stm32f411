/*
 * radio_type.h
 *
 *  Created on: Sep 20, 2022
 *      Author: tantrum
 */

#ifndef INC_RADIO_TYPE_H_
#define INC_RADIO_TYPE_H_

/*

typedef union{
	uint8_t ar_press[4];
	uint32_t press_Pa;
}pressure;

*/

typedef enum nrf_bytes{
	TEST_BYTE,
	LEFT_MOTOR,
	RIGHT_MOTOR,
	DEPHT_OFFSET,
	CAMERA_COMMAND,


	LEFT_BUT







} radio_bytes_to_diver;

typedef enum nrf_ack_bytes{
	TEST_BYTE_ACK,
	TEMP_INSIDE_MSB,
	TEMP_INSIDE_LSB,
	PRESSURE_INSIDE_0,
	PRESSURE_INSIDE_1,
	PRESSURE_INSIDE_2,
	PRESSURE_INSIDE_3,
	PISTON_OFFSET,
	BATTERY_VOLTAGE_MSB,
	BATTERY_VOLTAGE_LSB,
	X_AXIS_MSB,
	X_AXIS_LSB,
	Y_AXIS_MSB,
	Y_AXIS_LSB





} radio_bytes_to_console;


#endif /* INC_RADIO_TYPE_H_ */
