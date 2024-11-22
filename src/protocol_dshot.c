/*
 * dshot.c
 *
 *  Created on: Sep 28, 2023
 *      Author: hooke
 */

#include <protocol_dshot.h>
#include <assert.h>
#include <memory.h>

#define MHZ_TO_HZ(x) 			((x) * 1000000)
#define DSHOT1200_HZ     		MHZ_TO_HZ(24)
#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT150_HZ     		MHZ_TO_HZ(3)
#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20
#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

/*
11 bit throttle: 
	- 2048 possible values. 
	- 0 means disarmed. 
	- 1-47 reserved for special commands
1 bit telemetry request: telemety data is sent back by seperate channel
4 bit CRC: Cycle redundancy to validate data
Frame: SSSSSSSSSSSTCCCC
*/

/* Forward Declarations */
//void DShot_DMA_Callback(DMA_HandleTypeDef *hdma);
//uint16_t DShot_Prepare_Packet(uint16_t value);
//void DShot_Prepare_DMA_Buffer(uint32_t *motor_dmabuffer, uint16_t value);

void dshot_init(struct dshot_motor *dshot, int clockFreqInHz, enum dshot_type type)
{
	dshot->buffer = (uint32_t*)calloc(DSHOT_DMA_BUFFER_SIZE, sizeof(uint32_t));
	
	uint32_t freq = 0;
	switch (type)
	{
		case(DSHOT600):
				freq = DSHOT600_HZ;
		break;
		case(DSHOT300):
				freq = DSHOT300_HZ;
		break;
		case(DSHOT1200):
				freq = DSHOT1200_HZ;
		break;
		case(DSHOT150):
		default:
			freq = DSHOT150_HZ;
	}
	
	// calculate prescaler
	dshot->prescaler = lrintf((float) clockFreqInHz / freq + 0.01f) - 1;
	dshot->bitLengh = MOTOR_BITLENGTH;

	/*TIM_HandleTypeDef* tim = HandleStruct->Init.TIMInstance;
	switch (HandleStruct->Init.TIMChannel)
	{
		case TIM_CHANNEL_1:
			HandleStruct->DMAInstance = tim->hdma[TIM_DMA_ID_CC1];
			HandleStruct->DMACCR = (uint32_t)&tim->Instance->CCR1;
			HandleStruct->TIMCCR = TIM_DMA_CC1;
			break;
		case TIM_CHANNEL_2:
			HandleStruct->DMAInstance = tim->hdma[TIM_DMA_ID_CC2];
			HandleStruct->DMACCR = (uint32_t)&tim->Instance->CCR2;
			HandleStruct->TIMCCR = TIM_DMA_CC2;
			break;
		case TIM_CHANNEL_3:
			HandleStruct->DMAInstance = tim->hdma[TIM_DMA_ID_CC3];
			HandleStruct->DMACCR = (uint32_t)&tim->Instance->CCR3;
			HandleStruct->TIMCCR = TIM_DMA_CC3;
			break;
		case TIM_CHANNEL_4:
			HandleStruct->DMAInstance = tim->hdma[TIM_DMA_ID_CC4];
			HandleStruct->DMACCR = (uint32_t)&tim->Instance->CCR4;
			HandleStruct->TIMCCR = TIM_DMA_CC4;
			break;
	}*/

	//HandleStruct->DMABuffer = (uint32_t)calloc(DSHOT_DMA_BUFFER_SIZE, sizeof(uint32_t));

	// Setup the frequency
	

	// Calculate prescaler
	//uint16_t prescaler = lrintf((float) SystemCoreClock / freq + 0.01f) - 1;
	//__HAL_TIM_SET_PRESCALER(tim, prescaler);
	//__HAL_TIM_SET_AUTORELOAD(tim, MOTOR_BITLENGTH);

	// Setup callback (TIM_DMA_ID_CCx depends on timer channel)
	//HandleStruct->DMAInstance->XferCpltCallback = DShot_DMA_Callback;

	// Start PWM
	//AL_TIM_PWM_Start(tim, HandleStruct->Init.TIMChannel);
}

/*void DShot_Write_All(DShot_Handle_TypeDef* handle, int count)
{
	for (int i = 0; i < count; i++)
	{
		DShot_Prepare_DMA_Buffer((uint32_t*)handle[i].DMABuffer, handle[i].Value);
	}

	for (int i = 0; i < count; i++)
	{
		HAL_DMA_Start_IT(handle[i].DMAInstance, handle[i].DMABuffer, handle[i].DMACCR, DSHOT_DMA_BUFFER_SIZE);
	}

	for (int i = 0; i < count; i++)
	{
		__HAL_TIM_ENABLE_DMA(handle[i].Init.TIMInstance, handle[i].TIMCCR);
	}
}*/

// __HAL_TIM_DISABLE_DMA is needed to eliminate the delay between different dshot signals
/*void DShot_DMA_Callback(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}*/

void dshot_get_info(uint16_t value)
{
	// eRPM range
	assert(value == 0x0fff);
}

void dshot_write(struct dshot_motor *dshot, uint16_t value, bool telemetry)
{
	uint16_t packet;
	packet = (value << 1) | (telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	for(int i = 0; i < 16; i++)
	{
		dshot->buffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	dshot->buffer[16] = 0;
	dshot->buffer[17] = 0;

	dshot_protocol_write(&dshot->proto, dshot->buffer, sizeof(*dshot->buffer));
}

void dshot_protocol_read(struct dshot_protocol* proto, uint32_t* buf, uint32_t len)
{
	proto->read(proto->context, buf, len);
}

void dshot_protocol_write(struct dshot_protocol* proto, uint32_t* buf, uint32_t len)
{
	proto->write(proto->context, buf, len);
	//while (HAL_DMA_Start_IT(handle->DMAInstance, handle->DMABuffer, handle->DMACCR, DSHOT_DMA_BUFFER_SIZE) == HAL_BUSY)
	//__HAL_TIM_ENABLE_DMA(handle->Init.TIMInstance, handle->TIMCCR);
}

uint16_t dshot_prepare_packet(struct dshot_motor *dshot, uint16_t value)
{
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	if (dshot->useTelemetry) {
        csum = ~csum;
    }

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

// Convert 16 bits packet to 16 pwm signal
void dshot_prepare_buffer(struct dshot_motor *dshot, uint32_t* buffer, uint16_t value)
{
	uint16_t packet;
	packet = dshot_prepare_packet(dshot, value);

	for(int i = 0; i < 16; i++)
	{
		buffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	buffer[16] = 0;
	buffer[17] = 0;
}

void dshot_init_telementry(struct dshot_motor *dshot)
{

}