/*
 * mpu6050.c
 *
 *  Created on: Nov 11, 2021
 *      Author: LYH
 */

#include "mpu6050.h"

uint8_t timeout = 100;
uint32_t timer = 0;
double R = 0;
double Correction_Gyro_X_AngVel = 0, Correction_Gyro_Y_AngVel = 0, Correction_Gyro_Z_AngVel = 0;

KalmanFilter_HandleTypeDef KalmanFilter_X = {
    .Q_angle = 0.01f,
    .Q_gyro = 0.01f,
    .R_angle = 0.003f
};

KalmanFilter_HandleTypeDef KalmanFilter_Y = {
    .Q_angle = 0.01f,
    .Q_gyro = 0.01f,
    .R_angle = 0.003f
};

uint8_t mpu6050_Init_Correction(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct)
{
    uint8_t device_ID = 0;
    uint8_t data_buffer = 0x00;

    HAL_I2C_Mem_Read(hi2c, MPU6050_DEV_ADDR, WHO_AM_I, 1, &device_ID, 1, timeout);

    if (device_ID == 104)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay (200);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay (200);

    	data_buffer = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, PWR_MGMT_1, 1, &data_buffer, 1, timeout);  // PWR_MGMT_1 register

        data_buffer = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, SMPLRT_DIV, 1, &data_buffer, 1, timeout);  // SMPLRT_DIV register

        data_buffer = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, ACCEL_CONFIG, 1, &data_buffer, 1, timeout);  // ACCEL_CONFIG register

        data_buffer = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, GYRO_CONFIG, 1, &data_buffer, 1, timeout);  // GYRO_CONFIG register

        mpu6050_GyroCorrection(hi2c, data_struct);

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay (200);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay (200);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay (200);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay (200);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        return 0;
    }
    return 1;
}

void mpu6050_GyroCorrection(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct)
{
    uint8_t raw_data[6] = {0};
    uint8_t count = 0;

	while(count < 50)
	{
	    HAL_I2C_Mem_Read(hi2c, MPU6050_DEV_ADDR, GYRO_XOUT_H, 1, raw_data, 6, timeout);

	    data_struct->Gyro_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	    data_struct->Gyro_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data[3]);
	    data_struct->Gyro_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data[5]);

	    data_struct->Gyro_X_AngVel = data_struct->Gyro_X_RAW / 131.0;
	    data_struct->Gyro_Y_AngVel = data_struct->Gyro_Y_RAW / 131.0;
	    data_struct->Gyro_Z_AngVel = data_struct->Gyro_Z_RAW / 131.0;

	    Correction_Gyro_X_AngVel += data_struct->Gyro_X_AngVel;
	    Correction_Gyro_Y_AngVel += data_struct->Gyro_Y_AngVel;
	    Correction_Gyro_Z_AngVel += data_struct->Gyro_Z_AngVel;
	    count ++;
	    HAL_Delay (100);
	    //printf("count = %d  ,  ", count);
	    //printf("Correction_Gyro_X_AngVel = %f  ,  ", Correction_Gyro_X_AngVel);
	    //printf("Correction_Gyro_Y_AngVel = %f  ,  ", Correction_Gyro_Y_AngVel);
	    //printf("Correction_Gyro_Z_AngVel = %f\r\n", Correction_Gyro_Z_AngVel);
	}

	Correction_Gyro_X_AngVel /= 50.0;
	Correction_Gyro_Y_AngVel /= 50.0;
	Correction_Gyro_Z_AngVel /= 50.0;
	//printf("Correction_Gyro_X_AngVel = %f  ,  ", Correction_Gyro_X_AngVel);
	//printf("Correction_Gyro_Y_AngVel = %f  ,  ", Correction_Gyro_Y_AngVel);
	//printf("Correction_Gyro_Z_AngVel = %f\r\n", Correction_Gyro_Z_AngVel);
}

void mpu6050_AccelRead_Angle(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct)
{
    uint8_t raw_data[6] = {0};

    HAL_I2C_Mem_Read(hi2c, MPU6050_DEV_ADDR, ACCEL_XOUT_H, 1, raw_data, 6, timeout);

    data_struct->Accel_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    data_struct->Accel_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    data_struct->Accel_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    data_struct->Accel_X = data_struct->Accel_X_RAW / 16384.0;
    data_struct->Accel_Y = data_struct->Accel_Y_RAW / 16384.0;
    data_struct->Accel_Z = data_struct->Accel_Z_RAW / 16384.0;

    R = sqrt((data_struct->Accel_X)*(data_struct->Accel_X) + (data_struct->Accel_Y)*(data_struct->Accel_Y) + (data_struct->Accel_Z)*(data_struct->Accel_Z));
    data_struct->Accel_X_Angle = (acos((data_struct->Accel_X)/R)*57.2957795131);
    data_struct->Accel_Y_Angle = (acos((data_struct->Accel_Y)/R)*57.2957795131);
    data_struct->Accel_Z_Angle = (acos((data_struct->Accel_Z)/R)*57.2957795131);

    //printf("Accel_X_Angle = %f  ,  ", data_struct->Accel_X_Angle);
    //printf("Accel_Y_Angle = %f  ,  ", data_struct->Accel_Y_Angle);
    //printf("Accel_Z_Angle = %f\r\n", data_struct->Accel_Z_Angle);
}

void mpu6050_GyroRead_AngVel(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct)
{
    uint8_t raw_data[6] = {0};

    HAL_I2C_Mem_Read(hi2c, MPU6050_DEV_ADDR, GYRO_XOUT_H, 1, raw_data, 6, timeout);

    data_struct->Gyro_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    data_struct->Gyro_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    data_struct->Gyro_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    data_struct->Gyro_X_AngVel = (data_struct->Gyro_X_RAW / 131.0) - Correction_Gyro_X_AngVel;
    data_struct->Gyro_Y_AngVel = (data_struct->Gyro_Y_RAW / 131.0) - Correction_Gyro_Y_AngVel;
    data_struct->Gyro_Z_AngVel = (data_struct->Gyro_Z_RAW / 131.0) - Correction_Gyro_Z_AngVel;

    //printf("Gyro_X_AngVel = %f  ,  ", data_struct->Gyro_X_AngVel);
    //printf("Gyro_Y_AngVel = %f  ,  ", data_struct->Gyro_Y_AngVel);
    //printf("Gyro_Z_AngVel = %f\r\n", data_struct->Gyro_Z_AngVel);
}

void KalmanFilter_getAngle(I2C_HandleTypeDef *hi2c, MPU6050_HandleTypeDef *data_struct)
{
    uint8_t raw_data[14] = {0};
    int16_t temperature_RAW = 0;

    HAL_I2C_Mem_Read(hi2c, MPU6050_DEV_ADDR, ACCEL_XOUT_H, 1, raw_data, 14, timeout);

    // Accelerometer ============================================================
    data_struct->Accel_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    data_struct->Accel_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    data_struct->Accel_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    data_struct->Accel_X = data_struct->Accel_X_RAW / 16384.0;
    data_struct->Accel_Y = data_struct->Accel_Y_RAW / 16384.0;
    data_struct->Accel_Z = data_struct->Accel_Z_RAW / 16384.0;

    R = sqrt((data_struct->Accel_X)*(data_struct->Accel_X) + (data_struct->Accel_Y)*(data_struct->Accel_Y) + (data_struct->Accel_Z)*(data_struct->Accel_Z));
    data_struct->Accel_X_Angle = (acos((data_struct->Accel_X)/R)*57.2957795131);
    data_struct->Accel_Y_Angle = (acos((data_struct->Accel_Y)/R)*57.2957795131);
    data_struct->Accel_Z_Angle = (acos((data_struct->Accel_Z)/R)*57.2957795131);

    //printf("Accel_X_Angle = %f  ,  ", data_struct->Accel_X_Angle);
    //printf("Accel_Y_Angle = %f  ,  ", data_struct->Accel_Y_Angle);
    //printf("Accel_Z_Angle = %f  ,  ", data_struct->Accel_Z_Angle);

    // Temperature ============================================================
    temperature_RAW = (int16_t)(raw_data[6] << 8 | raw_data[7]);

    // Gyroscope (Angular Velocity Meter) ============================================================
    data_struct->Gyro_X_RAW = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    data_struct->Gyro_Y_RAW = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    data_struct->Gyro_Z_RAW = (int16_t)(raw_data[12] << 8 | raw_data[13]);

    data_struct->Gyro_X_AngVel = (data_struct->Gyro_X_RAW / 131.0) - Correction_Gyro_X_AngVel;
    data_struct->Gyro_Y_AngVel = (data_struct->Gyro_Y_RAW / 131.0) - Correction_Gyro_Y_AngVel;
    data_struct->Gyro_Z_AngVel = (data_struct->Gyro_Z_RAW / 131.0) - Correction_Gyro_Z_AngVel;

    //printf("Gyro_X_AngVel = %f  ,  ", data_struct->Gyro_X_AngVel);
    //printf("Gyro_Y_AngVel = %f  ,  ", data_struct->Gyro_Y_AngVel);
    //printf("Gyro_Z_AngVel = %f\r\n", data_struct->Gyro_Z_AngVel);

    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    data_struct->KalmanFilter_AngleX = 90 - KalmanFilter_Algorithm(&KalmanFilter_X, data_struct->Accel_X_Angle, data_struct->Gyro_Y_AngVel, dt);
    data_struct->KalmanFilter_AngleY = 90 - KalmanFilter_Algorithm(&KalmanFilter_Y, data_struct->Accel_Y_Angle, data_struct->Gyro_X_AngVel, dt);

    //printf("KalmanFilter_AngleX = %f , ", data_struct->KalmanFilter_AngleX);
    //printf("KalmanFilter_AngleY = %f\n", data_struct->KalmanFilter_AngleY);
}

double KalmanFilter_Algorithm(KalmanFilter_HandleTypeDef *KalmanFilter, double Angle, double AngVel, double dt)
{
    double Kalman_gain[2];

    // 1
    KalmanFilter->Angle += (AngVel - KalmanFilter->Q_bias) * dt;

    // 2
    KalmanFilter->P[0][0] += (KalmanFilter->Q_angle - KalmanFilter->P[0][1] - KalmanFilter->P[1][0] - KalmanFilter->P[1][1] * dt) * dt;
    KalmanFilter->P[0][1] += -(KalmanFilter->P[1][1] * dt);
    KalmanFilter->P[1][0] += -(KalmanFilter->P[1][1] * dt);
    KalmanFilter->P[1][1] += KalmanFilter->Q_gyro * dt;

    // 3
    Kalman_gain[0] = KalmanFilter->P[0][0] / (KalmanFilter->R_angle + KalmanFilter->P[0][0]);
    Kalman_gain[1] = KalmanFilter->P[1][0] / (KalmanFilter->R_angle + KalmanFilter->P[0][0]);

    // 4
    KalmanFilter->Angle += Kalman_gain[0] * (Angle - KalmanFilter->Angle);
    KalmanFilter->Q_bias += Kalman_gain[1] * (Angle - KalmanFilter->Angle);

    // 5
    KalmanFilter->P[0][0] -= Kalman_gain[0] * KalmanFilter->P[0][0];
    KalmanFilter->P[0][1] -= Kalman_gain[0] * KalmanFilter->P[0][1];
    KalmanFilter->P[1][0] -= Kalman_gain[1] * KalmanFilter->P[0][0];
    KalmanFilter->P[1][1] -= Kalman_gain[1] * KalmanFilter->P[0][1];

    return KalmanFilter->Angle;
}
