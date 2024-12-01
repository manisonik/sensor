/*
 * sensor.h
 *
 *  Created on: Dec 17, 2023
 *      Author: hooke
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA			0x68 	/* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB			0x69 	/* I2C slave address for INV device on Rev B board */

#define DEF_ST_ACCEL_FS                 2
#define DEF_ST_GYRO_FS_DPS              250
#define DEF_ST_SCALE                    32768
#define DEF_SELFTEST_GYRO_SENS			(DEF_ST_SCALE / DEF_ST_GYRO_FS_DPS)
#define DEF_ST_ACCEL_FS_MG				2000

#ifndef INV20948_ABS
#define INV20948_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

typedef int (*read_reg_t)(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
typedef int (*write_reg_t)(void* context, uint8_t reg, const uint8_t* buf, uint32_t len);
typedef void (*log_callback_t)(int level, const char * str, va_list ap);
typedef int (*delay_us_t)(uint16_t us);
typedef int (*delay_ms_t)(uint16_t ms);

typedef struct inv_invpres_init
{
	void *     context;
	log_callback_t log;
	read_reg_t read_callback;
	write_reg_t write_callback;
	delay_us_t delay_us;
	delay_ms_t delay_ms;
	int is_spi;
} inv_invpres_init_t;

/* data structure to hold pressure sensor related parameters */
typedef struct inv_invpres
{
	uint32_t min_delay_us;
	uint8_t pressure_en;
	uint8_t temperature_en;
	float sensor_constants[4]; // OTP values
	float p_Pa_calib[3];
	float LUT_lower;
	float LUT_upper;
	float quadr_factor;
	float offst_factor;
	inv_invpres_init_t init;
} inv_invpres_t;

int inv_invpres_init(struct inv_invpres *s);
int read_otp_from_i2c(struct inv_invpres *s, short *out);
void init_base(struct inv_invpres *s, short *otp);
int inv_invpres_process_data(struct inv_invpres *s,
		int p_LSB,
		int T_LSB,
		float *pressure,
		float *temperature
		);    
void calculate_conversion_constants(struct inv_invpres *s, float *p_Pa, float *p_LUT, float *out);
void initICM20948(inv_invpres_t* s);

#ifdef __cplusplus
}
#endif
#endif /* INC_SENSOR_H_ */
