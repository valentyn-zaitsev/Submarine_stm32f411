/*
 * vl6180.h
 *
 *  Created on: Aug 13, 2022
 *      Author: tantrum
 */

#ifndef INC_VL6180_H_
#define INC_VL6180_H_

#include "main.h"




uint8_t vl6180_read_byte(uint16_t reg);
void vl6180_write_byte(uint16_t reg,uint8_t data);
void vl6180_write_two_bytes(uint16_t reg,uint8_t data);
void vl6180_start_range(void);
void vl6180_poll_range(void);
void vl6180_clear_interrupts(void);
void vl6180_set_scaling(uint8_t new_scaling);
void vl6180_init(I2C_HandleTypeDef * i2c);
void vl6180_init_address(uint16_t addr);
void vl6180_change_addr(uint16_t newAddr);
uint16_t get_distance_mm(void);






#endif /* INC_VL6180_H_ */
