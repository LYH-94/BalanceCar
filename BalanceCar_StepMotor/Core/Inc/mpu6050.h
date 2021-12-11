/*
 * mpu6050.h
 *
 *  Created on: Nov 11, 2021
 *      Author: LYH
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_



#endif /* INC_MPU6050_H_ */

#include "main.h"
#include <math.h>

#define MPU6050_DEV_ADDR 		0xD0
#define WHO_AM_I 			0x75
#define PWR_MGMT_1 			0x6B
#define SMPLRT_DIV 			0x19
#define ACCEL_CONFIG 			0x1C
#define ACCEL_XOUT_H 			0x3B
#define GYRO_CONFIG 			0x1B
#define GYRO_XOUT_H 			0x43

typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Accel_X;
    double Accel_Y;
    double Accel_Z;
    double Accel_X_Angle;
    double Accel_Y_Angle;
    double Accel_Z_Angle;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gyro_X_AngVel;
    double Gyro_Y_AngVel;
    double Gyro_Z_AngVel;

    double KalmanFilter_AngleX;
    double KalmanFilter_AngleY;
} MPU6050_HandleTypeDef;

typedef struct
{
    double Q_angle;
    double Q_gyro;
    double R_angle;
    double Angle;
    double Q_bias;
    double P[2][2];
} KalmanFilter_HandleTypeDef;

uint8_t mpu6050_Init_Correction(I2C_HandleTypeDef *hi2c,  MPU6050_HandleTypeDef *data_struct);
void mpu6050_GyroCorrection(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct);

void mpu6050_AccelRead_Angle(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct);
void mpu6050_GyroRead_AngVel(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct);

void KalmanFilter_getAngle(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct);
double KalmanFilter_Algorithm(KalmanFilter_HandleTypeDef *KalmanFilter, double Angle, double AngVel, double dt);
