/*
 * vl6180.c
 *
 *  Created on: Aug 13, 2022
 *      Author: tantrum
 */

//SOURCE: https://github.com/leachj/vl6180_pi

#include "vl6180.h"

I2C_HandleTypeDef * i2c_interface;

uint8_t vl6180_read_byte(uint16_t reg){
    uint8_t data_write[2];
    uint8_t data_read = 0;
    data_write[0] = (reg >> 8) & 0xFF; // MSB of register address
    data_write[1] = reg & 0xFF; // LSB of register address

    HAL_I2C_Master_Transmit(i2c_interface, 0x52, data_write, 2, 1000);
    HAL_I2C_Master_Receive(i2c_interface, 0x52, &data_read, 1, 1000);

    return data_read;
}

void vl6180_write_byte(uint16_t reg,uint8_t data) {
	uint8_t data_write[3];
    data_write[0] = (reg >> 8) & 0xFF;; // MSB of register address
    data_write[1] = reg & 0xFF; // LSB of register address
    data_write[2] = data & 0xFF;
    HAL_I2C_Master_Transmit(i2c_interface, 0x52, data_write, 3, 1000);
}

void vl6180_write_two_bytes(uint16_t reg,uint8_t data) {
	uint8_t data_write[4];
    data_write[0] = (reg >> 8) & 0xFF;; // MSB of register address
    data_write[1] = reg & 0xFF; // LSB of register address
    data_write[2] = (data >> 8) & 0xFF;; // MSB of data
    data_write[3] = data & 0xFF; // LSB of data
    HAL_I2C_Master_Transmit(i2c_interface, 0x52, data_write, 4, 1000);
}

void vl6180_start_range(void) {
	vl6180_write_byte(0x018,0x01);
}

void vl6180_poll_range(void) {
	static uint8_t counter = 0;
    char status;
    char range_status;

    // check the status
    status = vl6180_read_byte(0x04f);
    range_status = status & 0x07;

    // wait for new measurement ready status
    while (range_status != 0x04) {
        status = vl6180_read_byte(0x04f);
        range_status = status & 0x07;
        counter++;
        if (counter > 10){
        	counter = 0;
        	break;
        }
    }
}

void vl6180_clear_interrupts(void) {
	vl6180_write_byte(0x015,0x07);
}

void vl6180_set_scaling(uint8_t new_scaling){

    uint8_t scalerValues[] = {0, 253, 127, 84};
    uint8_t defaultCrosstalkValidHeight = 20;
    if (new_scaling < 1 || new_scaling > 3) { return; }

    uint8_t ptp_offset = vl6180_read_byte(0x24);

    vl6180_write_two_bytes(0x96,scalerValues[new_scaling]);
    vl6180_write_byte(0x24,ptp_offset / new_scaling);
    vl6180_write_byte(0x21, defaultCrosstalkValidHeight / new_scaling);
    uint8_t rce = vl6180_read_byte(0x2d);
    vl6180_write_byte(0x2d, (rce & 0xFE) | (new_scaling == 1));
}

void vl6180_init(I2C_HandleTypeDef * i2c)
{
	i2c_interface = i2c;
	vl6180_init_address(0x52);
}

void vl6180_init_address(uint16_t addr){

    uint8_t setup = vl6180_read_byte(0x016);

    if(setup == 1){
    	vl6180_write_byte(0x0207, 0x01);
    	vl6180_write_byte(0x0208, 0x01);
        vl6180_write_byte(0x0096, 0x00);
        vl6180_write_byte(0x0097, 0xfd);
        vl6180_write_byte(0x00e3, 0x00);
        vl6180_write_byte(0x00e4, 0x04);
        vl6180_write_byte(0x00e5, 0x02);
        vl6180_write_byte(0x00e6, 0x01);
        vl6180_write_byte(0x00e7, 0x03);
        vl6180_write_byte(0x00f5, 0x02);
        vl6180_write_byte(0x00d9, 0x05);
        vl6180_write_byte(0x00db, 0xce);
        vl6180_write_byte(0x00dc, 0x03);
        vl6180_write_byte(0x00dd, 0xf8);
        vl6180_write_byte(0x009f, 0x00);
        vl6180_write_byte(0x00a3, 0x3c);
        vl6180_write_byte(0x00b7, 0x00);
        vl6180_write_byte(0x00bb, 0x3c);
        vl6180_write_byte(0x00b2, 0x09);
        vl6180_write_byte(0x00ca, 0x09);
        vl6180_write_byte(0x0198, 0x01);
        vl6180_write_byte(0x01b0, 0x17);
        vl6180_write_byte(0x01ad, 0x00);
        vl6180_write_byte(0x00ff, 0x05);
        vl6180_write_byte(0x0100, 0x05);
        vl6180_write_byte(0x0199, 0x05);
        vl6180_write_byte(0x01a6, 0x1b);
        vl6180_write_byte(0x01ac, 0x3e);
        vl6180_write_byte(0x01a7, 0x1f);
        vl6180_write_byte(0x0030, 0x00);

        // Recommended : Public registers - See data sheet for more detail

        vl6180_write_byte(0x0011, 0x10); // Enables polling for ‘New Sample ready’ when measurement completes
        vl6180_write_byte(0x010a, 0x30); // Set the averaging sample period (compromise between lower noise and increased execution time)
        vl6180_write_byte(0x003f, 0x46); // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
        vl6180_write_byte(0x0031, 0xFF); // sets the # of range measurements after which auto calibration of system is performed
        vl6180_write_byte(0x0040, 0x63); // Set ALS integration time to 100ms
        vl6180_write_byte(0x002e, 0x01); // perform a single temperature calibratio of the ranging sensor
        vl6180_write_byte(0x001b, 0x09); // Set default ranging inter-measurement period to 100ms
        vl6180_write_byte(0x003e, 0x31); // Set default ALS inter-measurement period to 500ms
        vl6180_write_byte(0x0014, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’

        vl6180_write_byte(0x016, 0x00);
    }

    vl6180_set_scaling(1);

}

void vl6180_change_addr(uint16_t newAddr)
{
    vl6180_write_byte(0x0212, newAddr);
}

uint16_t get_distance_mm(void){

    uint16_t range;
    vl6180_start_range();
    vl6180_poll_range();

    range = vl6180_read_byte(0x063);
    vl6180_clear_interrupts();
    return range;
}

