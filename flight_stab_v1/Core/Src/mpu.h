/*
 * mpu.h
 *
 *  Created on: Sep 22, 2020
 *      Author: d
 */

#ifndef MPU_H_
#define MPU_H_

#include <stdint.h>
#include "stm32f3xx_hal.h"

#define ABS(__v__) ((__v__<0)?(-__v__):(__v__))

#define MPU_I2C_ADDR 0xD0


/*#define MPU_REG_SMPLRT_DIV		0x19
#define MPU_REG_CONFIG			0x1A
#define MPU_REG_GYRO_CONFIG		0x1B
#define MPU_REG_ACCEL_CONFIG	0x1C
#define MPU_REG_FIFO_EN			0x23
#define MPU_REG_INT_PIN_CFG		0x37
#define MPU_REG_INT_ENABLE		0x38
#define MPU_REG_INT_STATUS		0x3A
#define MPU_REG_ACCEL_XOUT_H	0x3B
#define MPU_REG_ACCEL_XOUT_L	0x3C
#define MPU_REG_ACCEL_YOUT_H	0x3D
#define MPU_REG_ACCEL_YOUT_L	0x3E
#define MPU_REG_ACCEL_ZOUT_H	0x3F
#define MPU_REG_ACCEL_ZOUT_L	0x40
#define MPU_REG_TEMP_OUT_H		0x41
#define MPU_REG_TEMP_OUT_L		0x42
#define MPU_REG_GYRO_XOUT_H		0x43
#define MPU_REG_GYRO_XOUT_L		0x44
#define MPU_REG_GYRO_YOUT_H		0x45
#define MPU_REG_GYRO_YOUT_L		0x46
#define MPU_REG_GYRO_ZOUT_H		0x47
#define MPU_REG_GYRO_ZOUT_L		0x48
#define MPU_REG_USER_CTRL		0x6A
#define MPU_REG_PWR_MGMT_1		0x6B
#define MPU_REG_PWR_MGMT_2		0x6C
#define MPU_REG_FIFO_COUNTH		0x72
#define MPU_REG_FIFO_COUNTL		0x73
#define MPU_REG_FIFO_R_W		0x74
#define MPU_REG_WHO_AM_I		0x75*/


enum MPU_registers{
	MPU_REG_SMPLRT_DIV = 0x19,
	MPU_REG_CONFIG,
	MPU_REG_GYRO_CONFIG,
	MPU_REG_ACCEL_CONFIG,
	MPU_REG_FIFO_EN = 0x23,
	MPU_REG_INT_PIN_CFG = 0x37,
	MPU_REG_INT_ENABLE,
	MPU_REG_INT_STATUS = 0x3A,
	MPU_REG_ACCEL_XOUT_H,
	MPU_REG_ACCEL_XOUT_L,
	MPU_REG_ACCEL_YOUT_H,
	MPU_REG_ACCEL_YOUT_L,
	MPU_REG_ACCEL_ZOUT_H,
	MPU_REG_ACCEL_ZOUT_L,
	MPU_REG_TEMP_OUT_H,
	MPU_REG_TEMP_OUT_L,
	MPU_REG_GYRO_XOUT_H,
	MPU_REG_GYRO_XOUT_L,
	MPU_REG_GYRO_YOUT_H,
	MPU_REG_GYRO_YOUT_L,
	MPU_REG_GYRO_ZOUT_H,
	MPU_REG_GYRO_ZOUT_L,
	MPU_REG_USER_CTRL = 0x6A,
	MPU_REG_PWR_MGMT_1,
	MPU_REG_PWR_MGMT_2,
	MPU_REG_FIFO_COUNTH = 0x72,
	MPU_REG_FIFO_COUNTL,
	MPU_REG_FIFO_R_W,
	MPU_REG_WHO_AM_I
};

enum MPU_lpf_freq{
	LPF_FREQ_256,
	LPF_FREQ_188,
	LPF_FREQ_98,
	LPF_FREQ_42,
	LPF_FREQ_20,
	LPF_FREQ_10,
	LPF_FREQ_5
};

enum MPU_gyro_range{
	GYRO_RANGE_250_DEGS,
	GYRO_RANGE_500_DEGS,
	GYRO_RANGE_1000_DEGS,
	GYRO_RANGE_2000_DEGS
};

enum MPU_accel_range{
	ACCEL_RANGE_2G,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
};

enum MPU_fifo_cfg{
	FIFO_ENABLE = 64,
	FIFO_RESET = 4
};

enum MPU_power_opt{
	POWER_DISABLE_TEMP = 8,
	POWER_CYCLE = 32,
	POWER_SLEEP = 64,
	POWER_RESET = 128
};

enum MPU_wakeup_freq{
	WAKEUP_FREQ_1_25HZ,
	WAKEUP_FREQ_5HZ,
	WAKEUP_FREQ_20HZ,
	WAKEUP_FREQ_40HZ
};

enum MPU_clk_source{
	CLKSRC_INTERNAL_8MHZ,
	CLKSRC_GYRO_X,
	CLKSRC_GYRO_Y,
	CLKSRC_GYRO_Z,
	CLKSRC_STOP
};

enum MPU_fifo_en_sensors{
	FIFO_EN_TEMP = 128,
	FIFO_EN_XG = 64,
	FIFO_EN_YG = 32,
	FIFO_EN_ZG = 16,
	FIFO_EN_ACCEL = 8
};

enum MPU_int_pin_level{
	MPU_INT_ACT_HIGH,
	MPU_INT_ACT_LOW = 128
};

enum MPU_int_pin_pp_oc{
	MPU_INT_PIN_PUSHPULL,
	MPU_INT_PIN_OPEN = 64
};

enum MPU_int_pulse_hold{
	MPU_INT_PULSE,
	MPU_INT_HOLD = 32
};

enum MPU_int_read_clear{
	MPU_INT_CLEAR_READ_STATUS,
	MPU_INT_CLEAR_READ_ANY = 16
};

enum MPU_int_src_enable{
	MPU_INT_SRC_DATA_READY = 1,
	MPU_INT_SRC_FIFO_OF = 16
};

enum MPU_axis{
	MPU_AXIS_X = 1,
	MPU_AXIS_Y = 2,
	MPU_AXIS_Z = 4
};

typedef struct{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} mpu_raw_data_t;

typedef struct{
	float accel_x;
	float accel_y;
	float accel_z;
	float temp;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} mpu_norm_data_t;

class Mpu {
	public:
		Mpu(I2C_HandleTypeDef *i2c_handle);
		void mpu_set_samplerate_div(uint8_t div);
		void mpu_set_lowpass(MPU_lpf_freq freq);
		void mpu_set_gyro_range(MPU_gyro_range fsr);
		void mpu_set_accel_range(MPU_accel_range fsr);
		void mpu_set_fifo_enable(uint8_t fifo_en);
		void mpu_set_interrupt(uint8_t pincfg, uint8_t int_en_src);
		uint8_t mpu_get_int_status();
		void mpu_read_acc_raw(int16_t *acc);
		void mpu_read_acc_norm(float *acc);
		int16_t mpu_read_temp_raw();
		float mpu_read_temp_norm();
		void mpu_read_gyro_raw(int16_t *gyro);
		void mpu_read_gyro_raw_cal(int16_t *gyro);
		void mpu_read_gyro_norm(float *gyro);
		void mpu_read_all_raw(mpu_raw_data_t *data);
		void mpu_read_all_raw_cal(mpu_raw_data_t *data);
		void mpu_read_all_norm(mpu_norm_data_t *data);
		void mpu_set_fifo_cfg(uint8_t cfg);
		void mpu_set_power_cfg(uint8_t axis_disable, uint8_t power_opt, MPU_wakeup_freq lp_wakeup_freq, MPU_clk_source clksel);
		uint16_t mpu_get_fifo_count();
		uint8_t mpu_fifo_read();
		void mpu_fifo_write(uint8_t data);
		uint8_t mpu_check_whoami();
		void mpu_gyro_calibrate(const uint8_t sample_count, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


	private:
		HAL_StatusTypeDef mpu_write_byte(uint8_t addr, uint8_t dat);
		I2C_HandleTypeDef *mpu_i2c_handle;
		float mpu_gyro_scale;
		float mpu_accel_scale;

		int16_t mpu_gyro_offset[3] = {0, 0, 0};

		const float MPU_GYRO_SCALE[4] = {131.068, 65.534, 32.767, 16.3835};
		const float MPU_ACCEL_SCALE[4] = {16383.5, 8191.75, 4095.875, 2047.9375};
};

#endif /* MPU_H_ */
