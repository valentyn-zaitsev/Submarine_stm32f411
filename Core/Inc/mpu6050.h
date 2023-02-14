#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "main.h"

#define MPU6050_ADDR 			0xD0

#define RAD_TO_DEG 				57.295779513082320876798154814105

#define WHO_AM_I_REG 			0x75
#define PWR_MGMT_1_REG 			0x6B
#define SMPLRT_DIV_REG 			0x19
#define ACCEL_CONFIG_REG 		0x1C
#define ACCEL_XOUT_H_REG 		0x3B
#define TEMP_OUT_H_REG 			0x41
#define GYRO_CONFIG_REG 		0x1B
#define GYRO_XOUT_H_REG 		0x43


// mpu6050 structure
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} mpu6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_init(I2C_HandleTypeDef *i2c);

void MPU6050_read_accel(mpu6050_t *DataStruct);

void MPU6050_read_gyro(mpu6050_t *DataStruct);

void MPU6050_read_temp(mpu6050_t *DataStruct);

void MPU6050_read_all(mpu6050_t *DataStruct);

#endif
