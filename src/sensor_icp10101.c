#include "sensor_icp10101.h"

static inline int icp10101_serif_read_reg(invpres_serif_t* s,
	uint8_t reg, uint8_t* buf, uint32_t len)
{
	assert(s);

	if (len > s->max_read)
		return INVPRES_ERROR_SIZE;

	if (s->read_reg(s->context, reg, buf, len) != 0)
		return INVPRES_ERROR_TRANSPORT;

	return 0;
}

static inline int icp10101_serif_write_reg(invpres_serif_t* s,
	uint8_t reg, const uint8_t* buf, uint32_t len)
{
	assert(s);

	if (len > s->max_write)
		return INVPRES_ERROR_SIZE;

	if (s->write_reg(s->context, reg, buf, len) != 0)
		return INVPRES_ERROR_TRANSPORT;

	return 0;
}

int icp10101_init(struct inv_invpres *s)
{
	short otp[4];
	icp10101_read_otp_from_i2c(s, otp);
	icp10101_init_base(s, otp);
	return 0;
}

int icp10101_read_otp_from_i2c(struct inv_invpres *s, short *out)
{
	unsigned char data_write[10];
	unsigned char data_read[10] = { 0 };
	int status = INVPRES_ERROR_SUCCESS;
	int i;

	// OTP Read mode
	data_write[0] = 0xC5;
	data_write[1] = 0x95;
	data_write[2] = 0x00;
	data_write[3] = 0x66;
	data_write[4] = 0x9C;

	status = icp10101_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 5);
	if (status) {
		return status;
	}

	// Read OTP values
	for (i = 0; i < 4; i++)
	{
		data_write[0] = 0xC7;
		data_write[1] = 0xF7;
		status = icp10101_serif_write_reg(&s->serif, ICC_ADDR_PRS, data_write, 2);
		if (status) {
			return status;
		}
		status = icp10101_serif_read_reg(&s->serif, ICC_ADDR_PRS, data_read, 3);
		if (status) {
			return status;
		}
		out[i] = data_read[0] << 8 | data_read[1];
	}
	return 0;
}

void icp10101_init_base(struct inv_invpres *s, short *otp)
{
	for (int i = 0; i < 4; i++) {
		s->sensor_constants[i] = (float) otp[i];
	}

	s->p_Pa_calib[0] = 45000.0;
	s->p_Pa_calib[1] = 80000.0;
	s->p_Pa_calib[2] = 105000.0;
	s->LUT_lower = 3.5 * (1 << 20);
	s->LUT_upper = 11.5 * (1 << 20);
	s->quadr_factor = 1 / 16777216.0;
	s->offst_factor = 2048.0;
}

// p_LSB -- Raw pressure data from sensor
// T_LSB -- Raw temperature data from sensor
int icp10101_process_data(struct inv_invpres *s,
		int p_LSB,
		int T_LSB,
		float *pressure,
		float *temperature
		)
{
	float t;
	float s1, s2, s3;
	float in[3];
	float out[3];
	float A, B, C;
	t = (float) (T_LSB - 32768);
	s1 = s->LUT_lower + (float) (s->sensor_constants[0] * t * t) * s->quadr_factor;
	s2 = s->offst_factor * s->sensor_constants[3] + (float) (s->sensor_constants[1] * t * t) * s->quadr_factor;
	s3 = s->LUT_upper + (float) (s->sensor_constants[2] * t * t) * s->quadr_factor;
	in[0] = s1;
	in[1] = s2;
	in[2] = s3;
	calculate_conversion_constants(s, s->p_Pa_calib, in, out);
	A = out[0];
	B = out[1];
	C = out[2];
	*pressure = A + B / (C + p_LSB);
	*temperature = -45.f + 175.f / 65536.f * T_LSB;
	return 0;
}

// p_Pa -- List of 3 values corresponding to applied pressure in Pa
// p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
void icp10101_calculate_conversion_constants(struct inv_invpres *s, float *p_Pa, float *p_LUT, float *out)
{
	float A, B, C;
	C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) + p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) + p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) / (p_LUT[2] * (p_Pa[0] - p_Pa[1]) + p_LUT[0] * (p_Pa[1] - p_Pa[2]) + p_LUT[1] * (p_Pa[2] - p_Pa[0]));
	A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
	B = (p_Pa[0] - A) * (p_LUT[0] + C);
	out[0] = A;
	out[1] = B;
	out[2] = C;
}