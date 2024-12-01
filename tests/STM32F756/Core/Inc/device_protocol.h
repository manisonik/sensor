#ifndef __DEVICE_PROTOCOL_H__
#define __DEVICE_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "scpi/scpi.h"
#include "usbd_cdc_if.h"

#define SCPI_INPUT_BUFFER_LENGTH 256
#define SCPI_ERROR_QUEUE_SIZE 17
#define SCPI_IDN1 "Manisonik"
#define SCPI_IDN2 "INSTR2023"
#define SCPI_IDN3 NULL
#define SCPI_IDN4 "01-02"
#define SCPI_COMMAND_LENGTH 6
#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */
#define ICM_I2C_ADDR_REVA			0x68 	/* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB			0x69 	/* I2C slave address for INV device on Rev B board */

size_t SCPI_Write(scpi_t* context, const char* data, size_t len);
int SCPI_Error(scpi_t *context, int_fast16_t err);
scpi_result_t SCPI_Control(scpi_t *context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val);
scpi_result_t SCPI_Reset(scpi_t *context);
scpi_result_t SCPI_Flush(scpi_t *context);
scpi_result_t SCPI_IdnQ(scpi_t *context);
scpi_result_t SCPI_Rst(scpi_t *context);
scpi_result_t SCPI_Motor_All_On(scpi_t *context);
scpi_result_t SCPI_Motor_All_Off(scpi_t *context);

void SCPI_Initialize();
void SCPI_Run();

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_PROTOCOL_H__ */