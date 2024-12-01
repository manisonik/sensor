#include "device_protocol.h"

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];
scpi_t scpi_context;
bool allMotorsOn;

scpi_command_t scpi_commands[] =
{
	{ .pattern = "*IDN?", .callback = SCPI_IdnQ, },
	{ .pattern = "*RST", .callback = SCPI_Rst, },
	{ .pattern = "MOTOR:ALL:ON", .callback = SCPI_Motor_All_On },
	{ .pattern = "MOTOR:ALL:OFF", .callback = SCPI_Motor_All_Off },
	SCPI_CMD_LIST_END
};

scpi_interface_t scpi_interface =
{
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset
};

void SCPI_Initialize()
{
	SCPI_Init(&scpi_context,
		scpi_commands,
		&scpi_interface,
		scpi_units_def,
		SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
		scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
		scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE
	);
}

void SCPI_Run()
{   
    uint8_t rxData[8];
    memset(rxData, 0, 8);

    uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
    if (bytesAvailable > 0)
    {
      uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
      if (CDC_ReadRxBuffer_FS(rxData, bytesToRead)
          == USB_CDC_RX_BUFFER_OK)
      {
        SCPI_Input(&scpi_context, (const char*)rxData, bytesToRead);
      }
    }
}

scpi_result_t SCPI_Motor_All_On(scpi_t *context)
{
	(void) context;
	allMotorsOn = TRUE;
	printf("All motors enabled!\n");
	return SCPI_RES_OK;
}

scpi_result_t SCPI_Motor_All_Off(scpi_t *context)
{
	(void) context;
	allMotorsOn = FALSE;
	printf("All motors disabled!\n");
	return SCPI_RES_OK;
}

scpi_result_t SCPI_IdnQ(scpi_t *context)
{
	(void) context;
	return SCPI_RES_OK;
}

scpi_result_t SCPI_Rst(scpi_t *context)
{
	(void) context;

	return SCPI_RES_OK;
}

size_t SCPI_Write(scpi_t *context, const char *data, size_t len)
{
	(void) context;

	return 0;
}

scpi_result_t SCPI_Flush(scpi_t *context)
{
	(void) context;

	const char txData[] = "\n";
	while (CDC_Transmit_FS((uint8_t*)txData, strlen(txData)) == USBD_BUSY);

	return SCPI_RES_OK;
}

int SCPI_Error(scpi_t *context, int_fast16_t err)
{
	(void) context;

	return 0;
}

scpi_result_t SCPI_Control(scpi_t *context, scpi_ctrl_name_t ctrl,
		scpi_reg_val_t val)
{
	(void) context;

	return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t *context)
{
	(void) context;

	return SCPI_RES_OK;
}