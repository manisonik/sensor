/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "scpi/scpi.h"
#include "usbd_cdc_if.h"
#include "i2c.h"

/* InvenSense drivers and utils */
//#include "Icm20948.h"
//#include "Ak0991x.h"
//#include "SensorTypes.h"
//#include "SensorConfig.h"
//#include "InvScheduler.h"
//#include "RingByteBuffer.h"
//#include "Message.h"
//#include "ErrorHelper.h"
//#include "DataConverter.h"
//#include "RingBuffer.h"
//#include "DynProtocol.h"
//#include "DynProtocolTransportUart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
//scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];
//scpi_t scpi_context;
//bool allMotorsOn = FALSE;
//inv_icm20948_t icm_device;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cmdTask */
osThreadId_t cmdTaskHandle;
const osThreadAttr_t cmdTask_attributes = {
  .name = "cmdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/*size_t SCPI_Write(scpi_t* context, const char* data, size_t len);
int SCPI_Error(scpi_t *context, int_fast16_t err);
scpi_result_t SCPI_Control(scpi_t *context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val);
scpi_result_t SCPI_Reset(scpi_t *context);
scpi_result_t SCPI_Flush(scpi_t *context);
scpi_result_t SCPI_IdnQ(scpi_t *context);
scpi_result_t SCPI_Rst(scpi_t *context);
scpi_result_t SCPI_Motor_All_On(scpi_t *context);
scpi_result_t SCPI_Motor_All_Off(scpi_t *context);

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
};*/
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCmdTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /*SCPI_Init(&scpi_context,
			scpi_commands,
			&scpi_interface,
			scpi_units_def,
			SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
			scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
			scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE
			);*/
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of cmdTask */
  cmdTaskHandle = osThreadNew(StartCmdTask, NULL, &cmdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCmdTask */
/**
* @brief Function implementing the cmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdTask */
void StartCmdTask(void *argument)
{
  /* USER CODE BEGIN StartCmdTask */
  /* Infinite loop */
  for(;;)
  {
    /*uint8_t rxData[8];
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
    }*/
    osDelay(1);
  }
  /* USER CODE END StartCmdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*int idd_io_hal_read_reg(void* context, uint8_t reg, uint8_t* rbuffer, uint32_t rlen) {
	(void)context;

	HAL_I2C_Master_Transmit(&hi2c2, ICM_I2C_ADDR_REVB << 1, &reg, 1, 0xff);
	HAL_StatusTypeDef retVal = HAL_I2C_Master_Receive(&hi2c2, ICM_I2C_ADDR_REVB << 1, rbuffer, rlen, 2000);
	return retVal;
}

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen){
	(void)context;

  uint8_t data[wlen + 1];
	data[0] = reg;
	for (int i = 0; i < wlen; i++) {
		data[i + 1] = wbuffer[i];
	}

	HAL_StatusTypeDef retVal = HAL_I2C_Master_Transmit(&hi2c2, ICM_I2C_ADDR_REVB << 1, data, wlen + 1, 2000);
	return retVal;
}

scpi_result_t SCPI_Motor_All_On(scpi_t *context)
{
	(void) context;
	allMotorsOn = TRUE;
	return SCPI_RES_OK;
}

scpi_result_t SCPI_Motor_All_Off(scpi_t *context)
{
	(void) context;
	allMotorsOn = FALSE;
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
}*/
/* USER CODE END Application */

