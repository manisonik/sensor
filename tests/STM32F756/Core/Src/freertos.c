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
#include "stm32f7xx_hal.h"
#include "device_protocol.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "dma.h"
#include "sensor_icm20948.h"
//#include "sensor_icp10101.h"
#include "protocol_dshot.h"
#include "protocol_ibus.h"
#include "Fusion/Fusion.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE (100) // replace this with actual sample rate
#define FS_IA6B_DATA_BUFSIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Hardware timer */
hwtimer_t timHandle;

/* Ibus handlers */
ibus_t fsIa6b;
uint8_t fsIabRecvBuf[32];
uint8_t fsIabSendBuf[32];

ibus_t fsIa6bSensor;
uint8_t fsIabSensorRecvBuf[32];
uint8_t fsIabSensorSendBuf[32];

uint8_t recv;

/* Message queue object */
typedef struct 
{
  uint8_t Buf[32];
  uint8_t Idx;
} MSGQUEUE_OBJ_t;

FusionVector gyroscope = {0.0f, 0.0f, 0.0f}; // replace this with actual gyroscope data in degrees/s
FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
FusionVector magnetometer = {1.0f, 0.0f, 0.0f}; // replace this with actual magnetometer data in arbitrary units

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

// Define calibration
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f}; // same as bias
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f}; // same as bias
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Set AHRS algorithm settings
const FusionAhrsSettings settings = {
  .convention = FusionConventionNwu,
  .gain = 0.5f,
  .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
  .accelerationRejection = 10.0f,
  .magneticRejection = 10.0f,
  .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
  };

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for fsIA6Task */
osThreadId_t fsIA6TaskHandle;
const osThreadAttr_t fsIA6Task_attributes = {
  .name = "fsIA6Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fsIA6TelemetryT */
osThreadId_t fsIA6TelemetryTHandle;
const osThreadAttr_t fsIA6TelemetryT_attributes = {
  .name = "fsIA6TelemetryT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void log_message(int level, const char * str, va_list ap);
int i2c_read(void* context, uint8_t reg, uint8_t* rbuffer, uint32_t rlen);
int i2c_write(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
bool ibusReadData(uint8_t* data, uint32_t size);
bool ibusWriteData(uint8_t* data, uint32_t size);
bool ibusDataReceived(uint8_t* data, uint32_t size);
bool ibusReadDataSensor(uint8_t* data, uint32_t size);
bool ibusWriteDataSensor(uint8_t* data, uint32_t size);
bool ibusDataReceivedSensor(uint8_t* data, uint32_t size);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartFSIA6Task(void *argument);
void StartFSIA6TelemetryTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
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

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(MSGQUEUE_OBJ_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(MSGQUEUE_OBJ_t), &myQueue02_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of fsIA6Task */
  fsIA6TaskHandle = osThreadNew(StartFSIA6Task, NULL, &fsIA6Task_attributes);

  /* creation of fsIA6TelemetryT */
  fsIA6TelemetryTHandle = osThreadNew(StartFSIA6TelemetryTask, NULL, &fsIA6TelemetryT_attributes);

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
  // inv_invpres_t s;
  // s.init.context = 0; /* no need */
  // s.init.read_callback  = i2c_read;
  // s.init.write_callback = i2c_write;
  // s.init.delay_ms = delay_ms;
  // s.init.delay_us = delay_us;
  // s.init.log = log_message;
  // s.init.is_spi = false;
  // initICM20948(&s);
  
  // FusionOffsetInitialise(&offset, SAMPLE_RATE);
  // FusionAhrsInitialise(&ahrs);
  // FusionAhrsSetSettings(&ahrs, &settings);

  /* Infinite loop */
  for(;;)
  {
    /*portDISABLE_INTERRUPTS();
    HAL_GPIO_WritePin(TST_GPIO_Port, TST_Pin, GPIO_PIN_SET);
    delayUs(&timHandle, 100);
    HAL_GPIO_WritePin(TST_GPIO_Port, TST_Pin, GPIO_PIN_RESET);
    portENABLE_INTERRUPTS();*/

    //icm20948_poll_data();

    //__HAL_UART_CLEAR_PEFLAG(&huart1);

    // Acquire latest sensor data
    /*const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
    
    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    
    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static clock_t previousTimestamp;
    const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
    previousTimestamp = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

    // Print algorithm outputs
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
               earth.axis.x, earth.axis.y, earth.axis.z);*/

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartFSIA6Task */
/**
* @brief Function implementing the fsIA6Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFSIA6Task */
void StartFSIA6Task(void *argument)
{
  /* USER CODE BEGIN StartFSIA6Task */
  
  // Initialize ibus
  ibusConfig_t fsIa6bConfig;
  fsIa6bConfig.ReadData = ibusReadData;
  fsIa6bConfig.WriteData = ibusWriteData;
  fsIa6bConfig.ReceivedData = ibusDataReceived;
  ibusInit(&fsIa6b, &fsIa6bConfig);

  MSGQUEUE_OBJ_t msg;
  osStatus_t status;

  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(myQueue02Handle, &msg, NULL, 0U);   // wait for message
    if (status == osOK)
    {
       // Print channels
      printf("CH%d: %d", 0, (int)fsIa6b.ibusChannelData[0]);
      for (int i = 1; i < IBUS_MAX_CHANNEL; i++) {
        printf(", CH%d: %d", i, (int)fsIa6b.ibusChannelData[i]);
      }
      printf("\n");
    }
    osDelay(1);
  }
  /* USER CODE END StartFSIA6Task */
}

/* USER CODE BEGIN Header_StartFSIA6TelemetryTask */
/**
* @brief Function implementing the fsIA6TelemetryT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFSIA6TelemetryTask */
void StartFSIA6TelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartFSIA6TelemetryTask */

  // Initialize ibus
  ibusConfig_t fsIa6bConfig;
  fsIa6bConfig.ReadData = ibusReadDataSensor;
  fsIa6bConfig.WriteData = ibusWriteDataSensor;
  fsIa6bConfig.ReceivedData = ibusDataReceivedSensor;
  ibusInit(&fsIa6bSensor, &fsIa6bConfig);

  // Start receiving DMA (circular buffer)
  HAL_UART_Receive_DMA(&huart1, &recv, 1);

  MSGQUEUE_OBJ_t msg;
  osStatus_t status;

  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(myQueue01Handle, &msg, NULL, 0U);   // wait for message
    if (status == osOK)
    {
      ibusTelemetry(&fsIa6bSensor, msg.Buf);
    }
  }
  /* USER CODE END StartFSIA6TelemetryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t us = microsISR(&timHandle);
  if (huart->Instance == USART6) {
    ibusProcessDataISR(&fsIa6b, recv, us);
  } else if (huart->Instance == USART1) {
    ibusProcessDataISR(&fsIa6bSensor, recv, us);
  }
}

/// @brief 
/// @param huart 
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
    HAL_HalfDuplex_EnableReceiver(&huart1);
	}
}

/// @brief 
/// @param huart 
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
    printf("Half complete occurring\n");
	}
}

/// @brief 
/// @param huart 
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
	{
    printf("Abort complete!\n");
	}
}

/// @brief 
/// @param huart 
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
	{
    printf("Abort receive!\n");
	}
}

/// @brief 
/// @param huart 
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) 
  {
    if (huart->ErrorCode & HAL_UART_ERROR_PE) {
      // parity error
      printf("USART6 - Parity error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_NE) {
      // noise error
      printf("USART6 - Noise error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_FE) {
      // frame error
      printf("USART6 - Frame error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
      // clear overrun error
      __HAL_UART_CLEAR_OREFLAG(&huart6);

      // restart transmission
      HAL_HalfDuplex_EnableReceiver(&huart6);
      HAL_UART_Receive_DMA(&huart6, &recv, 1);
    }
    if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
      // dma error
      printf("USART6 - DMA error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_RTO) {
      // receive timeout error
      printf("USART6 - RTO error\n");
    }
  }
  else if (huart->Instance == USART1) 
  {
    if (huart->ErrorCode & HAL_UART_ERROR_PE) {
      // parity error
      printf("USART1 - Parity error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_NE) {
      // noise error
      printf("USART1 - Noise error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_FE) {
      // frame error
      printf("USART1 - Frame error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
      // clear overrun error
      __HAL_UART_CLEAR_OREFLAG(&huart1);

      // restart transmission
      HAL_HalfDuplex_EnableReceiver(&huart1);
      HAL_UART_Receive_DMA(&huart1, &recv, 1);
    }
    if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
      // dma error
      printf("USART1 - DMA error\n");
    }
    if (huart->ErrorCode & HAL_UART_ERROR_RTO) {
      // receive timeout error
      printf("USART1 - RTO error\n");
    }
  }
}

/// @brief 
/// @param data 
/// @param size 
/// @return 
bool ibusReadData(uint8_t* data, uint32_t size)
{
  HAL_HalfDuplex_EnableReceiver(&huart6);
  HAL_UART_Receive_DMA(&huart6, data, size);

  return true;
}

/// @brief 
/// @param data 
/// @param size 
/// @return 
bool ibusWriteData(uint8_t* data, uint32_t size)
{
  HAL_HalfDuplex_EnableTransmitter(&huart6);
  HAL_UART_Transmit_DMA(&huart6, data, size);

  return true;
}

/// @brief 
/// @param data 
/// @param size 
/// @return 
bool ibusReadDataSensor(uint8_t* data, uint32_t size)
{
  HAL_HalfDuplex_EnableReceiver(&huart1);
  HAL_UART_Receive_DMA(&huart1, &recv, size);
    
  return true;
}

/// @brief 
/// @param data 
/// @param size 
/// @return 
bool ibusWriteDataSensor(uint8_t* data, uint32_t size)
{
  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit_DMA(&huart1, data, size);

  return true;
}

/// @brief 
/// @param ibus 
/// @return 
bool ibusDataReceived(uint8_t* data, uint32_t size)
{
  MSGQUEUE_OBJ_t msg;
  msg.Idx = 0U;
  memcpy(msg.Buf, data, size);
  osMessageQueuePut(myQueue01Handle, &msg, 0U, 0U);

  return true;
}

/// @brief 
/// @param ibus 
/// @return 
bool ibusDataReceivedSensor(uint8_t* data, uint32_t size)
{
  MSGQUEUE_OBJ_t msg;
  msg.Idx = 0U;
  memcpy(msg.Buf, data, size);
  osMessageQueuePut(myQueue01Handle, &msg, 0U, 0U);

  return true;
}

/// @brief 
/// @param context 
/// @param reg 
/// @param rbuffer 
/// @param rlen 
/// @return 
int i2c_read(void* context, uint8_t reg, uint8_t* rbuffer, uint32_t rlen) {
	(void)context;

	HAL_I2C_Master_Transmit(&hi2c2, ICM_I2C_ADDR_REVB << 1, &reg, 1, 0xff);
	HAL_StatusTypeDef retVal = HAL_I2C_Master_Receive(&hi2c2, ICM_I2C_ADDR_REVB << 1, rbuffer, rlen, 2000);
	return retVal;
}

/// @brief 
/// @param context 
/// @param reg 
/// @param wbuffer 
/// @param wlen 
/// @return 
int i2c_write(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
	(void)context;

  uint8_t data[wlen + 1];
	data[0] = reg;
	for (uint32_t i = 0; i < wlen; i++) {
		data[i + 1] = wbuffer[i];
	}

	HAL_StatusTypeDef retVal = HAL_I2C_Master_Transmit(&hi2c2, ICM_I2C_ADDR_REVB << 1, data, wlen + 1, 2000);
	return retVal;
}

/// @brief Printer function for message facility
/// @param level 
/// @param str 
/// @param ap 
static void log_message(int level, const char * str, va_list ap) {
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[6] = {
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

	printf(ptr);
}

    /*static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;
    
    const uint32_t now = microsISR(&timHandle);

    int d = now - ibusTimeLast;
    if (d > IBUS_FRAME_GAP) {
       ibusFramePosition = 0;
       fsIa6bSensor.rxBytesToIgnore = 0;
    } else if (fsIa6bSensor.rxBytesToIgnore) {
        fsIa6bSensor.rxBytesToIgnore--;
        return;
    }

    ibusTimeLast = now;

    if (ibusFramePosition == 0) {
        if (ibusIsValidIa6bIbusPacketLength(c)) {
            fsIa6bSensor.ibusModel = IBUS_MODEL_IA6B;
            fsIa6bSensor.ibusSyncByte = c;
            fsIa6bSensor.ibusFrameSize = c;
            fsIa6bSensor.ibusChannelOffset = 2;
            fsIa6bSensor.ibusChecksum = 0xFFFF;
        } else if ((fsIa6bSensor.ibusSyncByte == 0) && (c == 0x55)) {
            fsIa6bSensor.ibusModel = IBUS_MODEL_IA6;
            fsIa6bSensor.ibusSyncByte = 0x55;
            fsIa6bSensor.ibusFrameSize = 31;
            fsIa6bSensor.ibusChecksum = 0x0000;
            fsIa6bSensor.ibusChannelOffset = 1;
        } else if ( fsIa6bSensor.ibusSyncByte != c) {
            return;
        }
    }

    fsIa6bSensor.data[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == fsIa6bSensor.ibusFrameSize - 1) {
        fsIa6bSensor.lastFrameTimeUs = now;
        fsIa6bSensor.ibusFrameDone = true;
       
        MSGQUEUE_OBJ_t msg;
        for (int i = 0; i < 32; i++) {
          msg.Buf[i] = fsIa6bSensor.data[i];
        }
        msg.Idx = 0U;
        osMessageQueuePut(myQueue01Handle, &msg, 0U, 0U);
    } else {
        ibusFramePosition++;
    }*/
/* USER CODE END Application */

