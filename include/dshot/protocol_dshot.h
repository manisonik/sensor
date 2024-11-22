/*
 * Dshot2.h
 *
 *  Created on: Oct 14, 2023
 *      Author: hooke
 */

#ifndef DSHOT_H_
#define DSHOT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**
  * @brief  DShot protocol enumeration
  * @note   Provides the various types of DShot protocols
  *
  */
typedef enum dshot_type
{
	DSHOT150,
	DSHOT300,
	DSHOT600,
	DSHOT1200
} dshot_type_t;

typedef enum dshot_beep
{
	DSHOT_BEEP_1,
	DSHOT_BEEP_2, 
	DSHOT_BEEP_3,
	DSHOT_BEEP_4,
	DSHOT_BEEP_5
} dshot_beep_t;

#define RPM_LSB 100.0f

struct dshot_protocol
{
	void* context;
	void(*write)(void* context, uint32_t* buf, uint32_t len);
	void(*read)(void* context, uint32_t* buf, uint32_t len);
};

/**
  * @brief  DShot Structure definition
  * @note   Structure used to configure the motor definition
  */
typedef struct dshot_motor
{
	uint32_t* buffer;
	uint16_t prescaler;
	uint16_t bitLengh;
	bool useTelemetry;
	struct dshot_protocol proto;
} dshot_motor_t;

typedef struct
{
	int32_t lo_word;
	int32_t hi_word;
} dshot_info_t;

void dshot_init(struct dshot_motor *dshot,  int clockFreqInHz, enum dshot_type type);
void dshot_init_telementry(struct dshot_motor *dshot);
void dshot_release(struct dshot_motor *dshot);
void dshot_write(struct dshot_motor *dshot, uint16_t value, bool telemetry);
void dshot_protocol_read(struct dshot_protocol* proto, uint32_t* buf, uint32_t len);
void dshot_protocol_write(struct dshot_protocol* proto, uint32_t* buf, uint32_t len);
uint16_t dshot_prepare_packet(struct dshot_motor *dshot, uint16_t value);
void dshot_prepare_buffer(struct dshot_motor *dshot, uint32_t* buffer, uint16_t value);
void dshot_get_info(uint16_t value);

/* special commands */
void dshot_beep(dshot_beep_t beep); // wait for 260 ms before next command
void dshot_esc_info();
void dshot_3d_mode(bool enable);
void dshot_get_settings();
void dshot_save_settings();
void dshot_extended_telemetry(bool enable);
void dshot_spin_direction(bool normal);
void dshot_led(int id, bool enable);
void dshot_audio_stream(bool enable);
void dshot_silent_mode(bool enable);
void dshot_sig_line_telementry(bool enable);
void dshot_sig_line_cont_erpm_telementry(bool enable);
void dshot_sig_line_cont_erpm_telementry_period(bool period);


//void DShot_Prepare_DMA_Buffer(uint32_t* buffer, uint16_t value);
//void DShot_Write_All(DShot_Handle_TypeDef* handle, int count);

#ifdef __cplusplus
}
#endif
#endif /* DSHOT_H_ */
