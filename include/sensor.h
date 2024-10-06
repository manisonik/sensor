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

typedef int (*read_reg_t)(void * context, uint8_t reg, uint8_t * buf, uint32_t len);
typedef int (*write_reg_t)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len);

void initICM20948(read_reg_t read_fn, write_reg_t write_fn);

#ifdef __cplusplus
}
#endif
#endif /* INC_SENSOR_H_ */
