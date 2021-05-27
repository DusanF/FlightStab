/*
 * mpu.cpp
 *
 *  Created on: Sep 22, 2020
 *      Author: d
 */

#include "mpu.h"
#include <math.h>


Mpu::Mpu(I2C_HandleTypeDef *i2c_handle){	//constructor
	mpu_i2c_handle = i2c_handle;
	mpu_gyro_scale = MPU_GYRO_SCALE[0];
	mpu_accel_scale = MPU_ACCEL_SCALE[0];
}

void Mpu::mpu_set_gyro_range(MPU_gyro_range fsr) {
	mpu_write_byte(MPU_REG_GYRO_CONFIG, (fsr<<3) & 0x18);
	mpu_gyro_scale = MPU_GYRO_SCALE[fsr];
}

void Mpu::mpu_set_accel_range(MPU_accel_range fsr) {
	mpu_write_byte(MPU_REG_ACCEL_CONFIG, (fsr<<3) & 0x18);
	mpu_accel_scale = MPU_ACCEL_SCALE[fsr];
}

void Mpu::mpu_set_fifo_enable(uint8_t fifo_en) {
	mpu_write_byte(MPU_REG_FIFO_EN, fifo_en);
}

void Mpu::mpu_set_interrupt(uint8_t pincfg, uint8_t int_en_src) {	//enable interrupt sources for mpu INT pin
	mpu_write_byte(MPU_REG_INT_PIN_CFG, pincfg);
	mpu_write_byte(MPU_REG_INT_ENABLE, int_en_src);
}

uint8_t Mpu::mpu_get_int_status() {
	uint8_t ret;

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &ret, 1, HAL_MAX_DELAY);
	return ret;
}

float Mpu::mpu_read_temp_norm() { 	//read temperature
	return ((float)mpu_read_temp_raw()/340) + 36.53;
}

void Mpu::mpu_read_acc_raw(int16_t *acc) {	//read raw accelerometer data
	uint8_t recv[6];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, recv, 6, HAL_MAX_DELAY);
	acc[0] = (recv[0] << 8) | recv[1];
	acc[1] = (recv[2] << 8) | recv[3];
	acc[2] = (recv[4] << 8) | recv[5];
}

void Mpu::mpu_read_gyro_raw(int16_t *gyro) {	//read raw gyro data
	uint8_t recv[6];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, recv, 6, HAL_MAX_DELAY);
	gyro[0] = (recv[0] << 8) | recv[1];
	gyro[1] = (recv[2] << 8) | recv[3];
	gyro[2] = (recv[4] << 8) | recv[5];
}

void Mpu::mpu_read_gyro_raw_cal(int16_t *gyro) {	//raw calibrated gyro
	uint8_t recv[6];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, recv, 6, HAL_MAX_DELAY);
	gyro[0] = ((recv[0] << 8) | recv[1]) - mpu_gyro_offset[0];
	gyro[1] = ((recv[2] << 8) | recv[3]) - mpu_gyro_offset[1];
	gyro[2] = ((recv[4] << 8) | recv[5]) - mpu_gyro_offset[2];
}

void Mpu::mpu_read_acc_norm(float *acc) {	//normalized accelerometer
	int16_t acc_raw[3];

	mpu_read_acc_raw(acc_raw);
	acc[0] = acc_raw[0] / mpu_accel_scale;
	acc[1] = acc_raw[1] / mpu_accel_scale;
	acc[2] = acc_raw[2] / mpu_accel_scale;
}

int16_t Mpu::mpu_read_temp_raw(){	//raw temperature
	uint8_t recv[2];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, recv, 2, HAL_MAX_DELAY);
	return((recv[0] << 8) | recv[1]);
}

void Mpu::mpu_read_gyro_norm(float *gyro) {// normalized, calibrated gyro data (deg/s)
	int16_t gyro_raw[3];

	mpu_read_gyro_raw_cal(gyro_raw);
	gyro[0] = gyro_raw[0] / mpu_gyro_scale;
	gyro[1] = gyro_raw[1] / mpu_gyro_scale;
	gyro[2] = gyro_raw[2] / mpu_gyro_scale;
}

void Mpu::mpu_read_all_raw(mpu_raw_data_t *data) {	//read both gyro and acc raw data
	uint8_t recv[14];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, recv, 14, HAL_MAX_DELAY);
	data->accel_x = (recv[0] << 8) | recv[1];
	data->accel_y = (recv[2] << 8) | recv[3];
	data->accel_z = (recv[4] << 8) | recv[5];
	data->temp = (recv[6] << 8) | recv[7];
	data->gyro_x = (recv[8] << 8) | recv[9];
	data->gyro_y = (recv[10] << 8) | recv[11];
	data->gyro_z = (recv[12] << 8) | recv[13];
}

void Mpu::mpu_read_all_raw_cal(mpu_raw_data_t *data) {
	uint8_t recv[14];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, recv, 14, HAL_MAX_DELAY);
	data->accel_x = (recv[0] << 8) | recv[1];
	data->accel_y = (recv[2] << 8) | recv[3];
	data->accel_z = (recv[4] << 8) | recv[5];
	data->temp = (recv[6] << 8) | recv[7];
	data->gyro_x = ((recv[8] << 8) | recv[9]) - mpu_gyro_offset[0];
	data->gyro_y = ((recv[10] << 8) | recv[11]) - mpu_gyro_offset[1];
	data->gyro_z = ((recv[12] << 8) | recv[13]) - mpu_gyro_offset[2];
}

void Mpu::mpu_read_all_norm(mpu_norm_data_t *data) {
	mpu_raw_data_t raw;

	mpu_read_all_raw_cal(&raw);
	data->accel_x = (float)raw.accel_x / mpu_accel_scale;
	data->accel_y = (float)raw.accel_y / mpu_accel_scale;
	data->accel_z = (float)raw.accel_z / mpu_accel_scale;
	data->temp = ((float)raw.temp / 340) + 36.53;
	data->gyro_x = (float)raw.gyro_x / mpu_gyro_scale;
	data->gyro_y = (float)raw.gyro_y / mpu_gyro_scale;
	data->gyro_z = (float)raw.gyro_z / mpu_gyro_scale;
}

void Mpu::mpu_set_fifo_cfg(uint8_t cfg) {
	mpu_write_byte(MPU_REG_USER_CTRL, cfg);
}

void Mpu::mpu_set_power_cfg(uint8_t axis_disable, uint8_t power_opt, MPU_wakeup_freq lp_wakeup_freq, MPU_clk_source clksel) {
	mpu_write_byte(MPU_REG_PWR_MGMT_1, (clksel & 0x07) | (power_opt & 0xE8));
	mpu_write_byte(MPU_REG_PWR_MGMT_2, (lp_wakeup_freq << 6) | (axis_disable & 0x3F));
}

uint16_t Mpu::mpu_get_fifo_count() {
	uint8_t recv[2];

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_FIFO_COUNTH, I2C_MEMADD_SIZE_8BIT, recv, 2, HAL_MAX_DELAY);
	return((recv[0] << 8) | recv[1]);
}

uint8_t Mpu::mpu_fifo_read() {
	uint8_t ret;

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_FIFO_R_W, I2C_MEMADD_SIZE_8BIT, &ret, 1, HAL_MAX_DELAY);
	return ret;
}

void Mpu::mpu_fifo_write(uint8_t data) {
	mpu_write_byte(MPU_REG_FIFO_R_W, data);
}

uint8_t Mpu::mpu_check_whoami() {	//check device ID (0/1)
	uint8_t whoami;

	HAL_I2C_Mem_Read(mpu_i2c_handle, MPU_I2C_ADDR, MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY);
	return whoami == (MPU_I2C_ADDR>>1);
}


void Mpu::mpu_set_samplerate_div(uint8_t div){
	mpu_write_byte(MPU_REG_SMPLRT_DIV, div);
}

void Mpu::mpu_set_lowpass(MPU_lpf_freq freq){
	mpu_write_byte(MPU_REG_CONFIG, freq & 7);
}

void Mpu::mpu_gyro_calibrate(const uint8_t sample_count, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){	//calibrate gyro drift
	int16_t raw[3];
	uint8_t samples = 0;
	int32_t gyro_bias_sum[3]={0,0,0};
	//mpu_raw_data_t raw;
	//int32_t acc_total;

	while(samples < sample_count){
		if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)){	//wait for interrupt, then read data
			mpu_read_gyro_raw(raw);
			gyro_bias_sum[0] += raw[0];
			gyro_bias_sum[1] += raw[1];
			gyro_bias_sum[2] += raw[2];
			samples++;

/*	read also accelerometer - calibrate only if vehicle is not moving
 * 	not stable, damaged sensor?
			mpu_read_all_raw(&raw);
			acc_total = sqrt((double)(raw.accel_x*raw.accel_x) + (raw.accel_y*raw.accel_y) + (raw.accel_z*raw.accel_z));
			if(acc_total > (4096-250) && acc_total < (4096+250)){
				gyro_bias_sum[0] += raw.gyro_x;
				gyro_bias_sum[1] += raw.gyro_y;
				gyro_bias_sum[2] += raw.gyro_z;
				samples++;
			}*/
		}
	}

	mpu_gyro_offset[0] = gyro_bias_sum[0] / sample_count;
	mpu_gyro_offset[1] = gyro_bias_sum[1] / sample_count;
	mpu_gyro_offset[2] = gyro_bias_sum[2] / sample_count;
}

HAL_StatusTypeDef Mpu::mpu_write_byte(uint8_t addr, uint8_t dat){
	uint8_t buf[2] = {addr, dat};
	return HAL_I2C_Master_Transmit(mpu_i2c_handle, MPU_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}
