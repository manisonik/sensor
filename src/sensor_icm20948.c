/*
 * sensor.c
 *
 *  Created on: Dec 17, 2023
 *      Author: hooke
 */

#include "sensor_icm20948.h"
#include "Icm20948.h"
#include "Icm20948MPUFifoControl.h"
#include "Icm20948Defs.h"
#include "Message.h"
#include "Ak0991x.h"
#include "SensorTypes.h"
#include "SensorConfig.h"

inv_icm20948_t icm_device;

static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
static int unscaled_bias[THREE_AXES * 2];

static const uint8_t dmp3_image[] =
{ "icm20948_img.dmp3a.h" };

/* FSR configurations */
int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

/*
* Printer function for message facility
*/
static void msg_printer(int level, const char * str, va_list ap) {
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	while(*ptr != '\0') {
		//usart_serial_putchar(DEBUG_UART, *ptr);
		++ptr;
	}
}

/*
* Mounting matrix configuration applied for Accel, Gyro and Mag
*/
static const float cfg_mounting_matrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

static uint8_t icm20948_get_grv_accuracy(void) {
	uint8_t accel_accuracy;
	uint8_t gyro_accuracy;

	accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
	gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
	return (min(accel_accuracy, gyro_accuracy));
}

static void icm20948_apply_mounting_matrix(void) {
	int ii;

	for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
		inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, ii);
	}
}

static void icm20948_set_fsr(void) {
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

void inv_icm20948_sleep(int ms) {
	//HAL_Delay(ms);
}

void inv_icm20948_sleep_us(int us) {
	//HAL_Delay(us/1000);
}

int load_dmp3(void) {
	int rc = 0;
	rc = inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));
	return rc;
}

int icm20948_sensor_setup(void) {
	int rc;
	uint8_t i, whoami = 0xff;

	/*
	 * Just get the whoami
	 */
	rc = inv_icm20948_get_whoami(&icm_device, &whoami);
	if (whoami == 0xff) { // if whoami fails try the other I2C Address
		//switch_I2C_to_revA();
		rc = inv_icm20948_get_whoami(&icm_device, &whoami);
	}

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for (i = 0; i < sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0]); ++i)
	{
		if (whoami == EXPECTED_WHOAMI[i])
		{
			break;
		}
	}

	if (i == sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0]))
	{
		//msg_printer(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x.", whoami);
		return rc;
	}

	/* Setup accel and gyro mounting matrix and associated angle for current board */
	inv_icm20948_init_matrix(&icm_device);

	/* set default power mode */
	//INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting Icm20948 in sleep mode...");
	rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
	if (rc != 0)
	{
		//msg_printer(INV_MSG_LEVEL_ERROR, "Initialization failed. Error loading DMP3...");
		return rc;
	}

	/*
	 * Configure and initialize the ICM20948 for normal use
	 */
	//msg_printer(INV_MSG_LEVEL_INFO, "Booting up icm20948...");

	/* Initialize auxiliary sensors */
	inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);
	rc = inv_icm20948_initialize_auxiliary(&icm_device);
	if (rc == -1)
	{
		//msg_printer(INV_MSG_LEVEL_ERROR, "Compass not detected...");
	}

	icm20948_apply_mounting_matrix();
	icm20948_set_fsr();

	/* re-initialize base state structure */
	inv_icm20948_init_structure(&icm_device);

	/* we should be good to go ! */
	//msg_printer(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	return 0;
}

void inv_icm20948_get_st_bias(struct inv_icm20948 * s, int *gyro_bias, int *accel_bias, int * st_bias, int * unscaled) {
	int axis, axis_sign;
	int gravity, gravity_scaled;
	int i, t;
	int check;
	int scale;

	/* check bias there ? */
	check = 0;
	for (i = 0; i < 3; i++) {
		if (gyro_bias[i] != 0)
			check = 1;
		if (accel_bias[i] != 0)
			check = 1;
	}

	/* if no bias, return all 0 */
	if (check == 0) {
		for (i = 0; i < 12; i++)
			st_bias[i] = 0;
		return;
	}

	/* dps scaled by 2^16 */
	scale = 65536 / DEF_SELFTEST_GYRO_SENS;

	/* Gyro normal mode */
	t = 0;
	for (i = 0; i < 3; i++) {
		st_bias[i + t] = gyro_bias[i] * scale;
		unscaled[i + t] = gyro_bias[i];
	}
	axis = 0;
	axis_sign = 1;
	if (INV20948_ABS(accel_bias[1]) > INV20948_ABS(accel_bias[0]))
		axis = 1;
	if (INV20948_ABS(accel_bias[2]) > INV20948_ABS(accel_bias[axis]))
		axis = 2;
	if (accel_bias[axis] < 0)
		axis_sign = -1;

	/* gee scaled by 2^16 */
	scale = 65536 / (DEF_ST_SCALE / (DEF_ST_ACCEL_FS_MG / 1000));

	gravity = 32768 / (DEF_ST_ACCEL_FS_MG / 1000) * axis_sign;
	gravity_scaled = gravity * scale;

	/* Accel normal mode */
	t += 3;
	for (i = 0; i < 3; i++) {
		st_bias[i + t] = accel_bias[i] * scale;
		unscaled[i + t] = accel_bias[i];
		if (axis == i) {
			st_bias[i + t] -= gravity_scaled;
			unscaled[i + t] -= gravity;
		}
	}
}

int icm20948_run_selftest(void) {
	static int rc = 0;		// Keep this value as we're only going to do this once.
	int gyro_bias_regular[THREE_AXES];
	int accel_bias_regular[THREE_AXES];
	static int raw_bias[THREE_AXES * 2];

	if (icm_device.selftest_done == 1) {
		//msg_printer(INV_MSG_LEVEL_INFO, "Self-test has already run. Skipping.");
	}
	else {
		/*
		* Perform self-test
		* For ICM20948 self-test is performed for both RAW_ACC/RAW_GYR
		*/
		//msg_printer(INV_MSG_LEVEL_INFO, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm20948_run_selftest(&icm_device, gyro_bias_regular, accel_bias_regular);
		if ((rc & INV_ICM20948_SELF_TEST_OK) == INV_ICM20948_SELF_TEST_OK) {
			/* On A+G+M self-test success, offset will be kept until reset */
			icm_device.selftest_done = 1;
			icm_device.offset_done = 0;
			rc = 0;
		} else {
			/* On A|G|M self-test failure, return Error */
			//msg_printer(INV_MSG_LEVEL_ERROR, "Self-test failure !");
			/* 0 would be considered OK, we want KO */
			rc = INV_ERROR;
		}

		/* It's advised to re-init the icm20948 device after self-test for normal use */
		icm20948_sensor_setup();
		inv_icm20948_get_st_bias(&icm_device, gyro_bias_regular, accel_bias_regular, raw_bias, unscaled_bias);
		//msg_printer(INV_MSG_LEVEL_INFO, "GYR bias (FS=250dps) (dps): x=%f, y=%f, z=%f", (float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
		//msg_printer(INV_MSG_LEVEL_INFO, "ACC bias (FS=2g) (g): x=%f, y=%f, z=%f", (float)(raw_bias[0 + 3] / (float)(1 << 16)), (float)(raw_bias[1 + 3] / (float)(1 << 16)), (float)(raw_bias[2 + 3] / (float)(1 << 16)));
	}

	return rc;
}

/*int hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen){
	(void)context;

	HAL_I2C_Master_Transmit(&hi2c2, ICM_I2C_ADDR_REVB << 1, &reg, 1, 0xff);

	HAL_StatusTypeDef retVal = HAL_I2C_Master_Receive(&hi2c2, ICM_I2C_ADDR_REVB << 1, rbuffer, rlen, 2000);

	return retVal;
}

int hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen){
	(void)context;

	uint8_t data[wlen + 1];
	data[0] = reg;
	for (int i = 0; i < wlen; i++) {
		data[i + 1] = wbuffer[i];
	}

	HAL_StatusTypeDef retVal = HAL_I2C_Master_Transmit(&hi2c2, ICM_I2C_ADDR_REVB << 1, data, wlen + 1, 2000);

	return retVal;
}*/

void initICM20948(inv_invpres_t* s) {
	int rc;

	/*
	 * Initialize icm20948 serif structure
	 */
	struct inv_icm20948_serif icm20948_serif;
	icm20948_serif.context = 0; /* no need */
	icm20948_serif.read_reg  = s->serif_read_reg;
	icm20948_serif.write_reg = s->serif_write_reg;
	icm20948_serif.max_read = 1024 * 16; /* maximum number of bytes allowed per serial read */
	icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */
	icm20948_serif.is_spi = false;

	/*
	 * Reset icm20948 driver states
	 */
	inv_icm20948_reset_states(&icm_device, &icm20948_serif);
	inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

	/*
	 * Setup the icm20948 device
	 */
	rc = icm20948_sensor_setup();

	/*
	 * Now that Icm20948 device was initialized, we can proceed with DMP image loading
	 * This step is mandatory as DMP image are not store in non volatile memory
	 */
	rc += load_dmp3();
	//check_rc(rc, "Error sensor_setup/DMP loading.");
}