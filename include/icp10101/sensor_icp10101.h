/*
 * Dshot2.h
 *
 *  Created on: Oct 14, 2023
 *      Author: hooke
 */

#ifndef ICP10101_H_
#define ICP10101_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICC_ADDR_PRS 0x63

enum inv_error
{
	INVPRES_ERROR_SUCCESS = 0,   /**< no error */
	INVPRES_ERROR = -1,  /**< unspecified error */
	INVPRES_ERROR_NIMPL = -2,  /**< function not implemented for given arguments */
	INVPRES_ERROR_TRANSPORT = -3,  /**< error occured at transport level */
	INVPRES_ERROR_TIMEOUT = -4,  /**< action did not complete in the expected time window */
	INVPRES_ERROR_SIZE = -5,  /**< size/length of given arguments is not suitable to complete requested action */
	INVPRES_ERROR_OS = -6,  /**< error related to OS */
	INVPRES_ERROR_IO = -7,  /**< error related to IO operation */
	INVPRES_ERROR_MEM = -9,  /**< not enough memory to complete requested action */
	INVPRES_ERROR_HW = -10, /**< error at HW level */
	INVPRES_ERROR_BAD_ARG = -11, /**< provided arguments are not good to perform requestion action */
	INVPRES_ERROR_UNEXPECTED = -12, /**< something unexpected happened */
	INVPRES_ERROR_FILE = -13, /**< cannot access file or unexpected format */
	INVPRES_ERROR_PATH = -14, /**< invalid file path */
	INVPRES_ERROR_IMAGE_TYPE = -15, /**< error when image type is not managed */
	INVPRES_ERROR_WATCHDOG = -16, /**< error when device doesn't respond to ping */
	INVPRES_ERROR_FIFO_OVERFLOW = -17, /**< FIFO overflow detected */
};

typedef struct invpres_serif
{
	int max_read;
	int max_write;
	void* context;
	int (*read_reg)(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
	int (*write_reg)(void* context, uint8_t reg, const uint8_t* buf, uint32_t len);
} invpres_serif_t;

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
	invpres_serif_t serif;
} inv_invpres_t;

int icp10101_init(struct inv_invpres *s);
int icp10101_read_otp_from_i2c(struct inv_invpres *s, short *out);
void icp10101_init_base(struct inv_invpres *s, short *otp);
int icp10101_process_data(struct inv_invpres *s,
		int p_LSB,
		int T_LSB,
		float *pressure,
		float *temperature
		);
void icp10101_calculate_conversion_constants(struct inv_invpres *s, float *p_Pa, float *p_LUT, float *out);

#ifdef __cplusplus
}
#endif
#endif /* ICP10101_H_ */
