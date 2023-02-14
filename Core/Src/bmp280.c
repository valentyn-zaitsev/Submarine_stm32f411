#include "bmp280.h"

I2C_HandleTypeDef * interface;

bmp280_calibration BMP280_CALIBRATION_DATA;

BME280_S32_t t_fine;
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
{
    BME280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((BME280_S32_t)BMP280_CALIBRATION_DATA.dig_T1<<1))) * ((BME280_S32_t)BMP280_CALIBRATION_DATA.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BME280_S32_t)BMP280_CALIBRATION_DATA.dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)BMP280_CALIBRATION_DATA.dig_T1))) >> 12) * ((BME280_S32_t)BMP280_CALIBRATION_DATA.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2, p;
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P4)<<35);
    var1 = ((var1 * var1 * (BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P3)>>8) + ((var1 * (BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P2)<<12);
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0;
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)BMP280_CALIBRATION_DATA.dig_P7)<<4);
    return (BME280_U32_t)p;
}


///////////////////////////////////////////////////////////////////


bmp280_calibration BMP280_CALIBRATION_DATA;

bmp280_calibration BMP280_GetCalibrationData(void)
{
    bmp280_calibration calibration_data;
    uint8_t data[24];
    BMP280_read(BMP280_REG_DIG_T1_LSB, data, sizeof(data));
    calibration_data.dig_T1 = (uint16_t)*(uint16_t *)&data[0];
    calibration_data.dig_T2 = (int16_t)*(int16_t *)&data[2];
    calibration_data.dig_T3 = (int16_t)*(int16_t *)&data[4];
    calibration_data.dig_P1 = (uint16_t)*(uint16_t *)&data[6];
    calibration_data.dig_P2 = (int16_t)*(int16_t *)&data[8];
    calibration_data.dig_P3 = (int16_t)*(int16_t *)&data[10];
    calibration_data.dig_P4 = (int16_t)*(int16_t *)&data[12];
    calibration_data.dig_P5 = (int16_t)*(int16_t *)&data[14];
    calibration_data.dig_P6 = (int16_t)*(int16_t *)&data[16];
    calibration_data.dig_P7 = (int16_t)*(int16_t *)&data[18];
    calibration_data.dig_P8 = (int16_t)*(int16_t *)&data[20];
    calibration_data.dig_P9 = (int16_t)*(int16_t *)&data[22];
    return calibration_data;
}

uint8_t BMP280_GetChipId(void)
{
    uint8_t data[1];
    BMP280_read(BMP280_REG_CHIP_ID, data, 1);
    return data[0];
}

int32_t BMP280_GetTemperature(void)
{
    uint8_t data[3];
    BMP280_read(BMP280_REG_TEMPERATURE, data, sizeof(data));
    int adc = (int32_t) ((((int32_t) (data[0])) << 12) | (((int32_t) (data[1])) << 4) | (((int32_t) (data[2])) >> 4));
    return BME280_compensate_T_int32(adc);
}

uint64_t BMP280_GetPressure(void)
{
    uint8_t data[3];
    BMP280_read(BMP280_REG_PRESSURE, data, sizeof(data));
    int adc = (int32_t) ((((int32_t) (data[0])) << 12) | (((int32_t) (data[1])) << 4) | (((int32_t) (data[2])) >> 4));
    return BME280_compensate_P_int64(adc);
}

void BMP280_Reset(void)
{
    uint8_t data = BMP280_RESET_VALUE;
    BMP280_write(BMP280_REG_RESET, &data, 1);
}

void BMP280_SetConfig(bmp280_config conf)
{
    uint8_t config_reg = ((uint8_t) (conf.inactive<<5)) | ((uint8_t) (conf.filter << 2));
    uint8_t ctrl_reg = ((uint8_t) (conf.osrs_p << 5)) | ((uint8_t) (conf.osrs_t << 2)) | ((uint8_t) conf.mode & 0b00000011);
    BMP280_write(BMP280_REG_CTRL_MEAS, &ctrl_reg, 1);
    BMP280_write(BMP280_REG_CONFIG, &config_reg, 1);
}

void BMP280_Init(I2C_HandleTypeDef * i)
{
    BMP280_IO_I2C(i);
    BMP280_CALIBRATION_DATA = BMP280_GetCalibrationData();
}


void BMP280_IO_I2C(I2C_HandleTypeDef * i)
{
    interface = i;
}

HAL_StatusTypeDef BMP280_read(unsigned char reg, unsigned char *buffer, unsigned short length)
{
    return HAL_I2C_Mem_Read(interface, 0x76<<1, reg, 1, buffer, length, 1000);
}

HAL_StatusTypeDef BMP280_write(unsigned char reg, unsigned char *buffer, unsigned short length)
{
    return HAL_I2C_Mem_Write(interface, 0x76<<1, reg, 1, buffer, length, 1000);
}


