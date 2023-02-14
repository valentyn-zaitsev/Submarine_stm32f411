/*
 * bmp280.h
 *
 *  Created on: Aug 26, 2022
 *      Author: tantrum
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdint.h>
//#include "i2c.h"
#include "main.h"

//Address map of BMP280 registers


#define BMP280_REG_DIG_T1_LSB 0x88
#define BMP280_REG_TEMPERATURE 0xFA
#define BMP280_REG_PRESSURE 0xF7
#define BMP280_REG_RESET 0x0E
#define BMP280_REG_CHIP_ID 0xD0
#define BMP280_REG_CTRL_MEAS 0xf4
#define BMP280_REG_CONFIG 0xf5


#define BMP280_RESET_VALUE 0xB6




#define BME280_S32_t int32_t
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t


BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P);


typedef struct
{
    unsigned short dig_T1;
    short dig_T2;
    short dig_T3;
    unsigned short dig_P1;
    short dig_P2;
    short dig_P3;
    short dig_P4;
    short dig_P5;
    short dig_P6;
    short dig_P7;
    short dig_P8;
    short dig_P9;
} bmp280_calibration;

//extern bmp280_calibration BMP280_CALIBRATION_DATA;

typedef enum {
    INACTIVE_05 = 0b000,
    INACTIVE_62_5 = 0b001,
    INACTIVE_125 = 0b010,
    INACTIVE_250 = 0b011,
    INACTIVE_500 = 0b100,
    INACTIVE_1000 = 0b101,
    INACTIVE_2000 = 0b110,
    INACTIVE_4000 = 0b111
} INACTIVE_INTERVAL;

typedef enum {
    FILTER_OFF = 0b000,
    FILTER_2 = 0b001,
    FILTER_4 = 0b010,
    FILTER_8 = 0b011,
    FILTER_16 = 0b100
} FILTER_CONSTANT;

typedef enum {
    OVERSAMPLING_SKIPPED = 0b000,
    OVERSAMPLING_1 = 0b001,
    OVERSAMPLING_2 = 0b010,
    OVERSAMPLING_4 = 0b011,
    OVERSAMPLING_8 = 0b100,
    OVERSAMPLING_16 = 0b101
} OVERSAMPLING;

typedef enum {
    MODE_SLEEP = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
} BMP280_MODE;

typedef struct
{
    INACTIVE_INTERVAL inactive;
    FILTER_CONSTANT filter;
    OVERSAMPLING osrs_p;
    OVERSAMPLING osrs_t;
    BMP280_MODE mode;
} bmp280_config;






void BMP280_Init(I2C_HandleTypeDef * interface);
void BMP280_Reset(void);
uint8_t BMP280_GetChipId(void);
int32_t BMP280_GetTemperature(void);
uint64_t BMP280_GetPressure(void);
void BMP280_SetConfig(bmp280_config config);
bmp280_calibration BMP280_GetCalibrationData(void);


HAL_StatusTypeDef BMP280_read(unsigned char reg, unsigned char * buffer, unsigned short length);
HAL_StatusTypeDef BMP280_write(unsigned char reg, unsigned char * buffer, unsigned short length);
void BMP280_IO_I2C(I2C_HandleTypeDef * i);










#endif /* INC_BMP280_H_ */
