cmake_minimum_required(VERSION 3.22)

set(stm32F756_ProjectName STM32F756)

# Create an executable object type
add_executable(${stm32F756_ProjectName}
	# HAL Core
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dma.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/eth.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/freertos.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/device_protocol.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/gpio.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/i2c.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f7xx_hal_msp.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f7xx_hal_timebase_tim.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f7xx_it.c"
	#"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c" # exclude for semihosting
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32f7xx.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tim.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/usart.c"

	# Drivers - STM32F7xx
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_eth.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c"

	# Middleware - USB CDC
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"

	# Middleware - USB Core
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"

	# Middleware - FreeRTOS
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/timers.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"

	# Device - USB
	"${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usb_device.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_cdc_if.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c"

	# Startup
	"${CMAKE_CURRENT_SOURCE_DIR}/Startup/startup_stm32f756zgtx.s"

	# eMD SmartMotion Driver
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Ak0991x/Ak0991x.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Setup.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Transport.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Sensor.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol/DynProtocol.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol/DynProtocolTransportUart.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/DataConverter.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/ErrorHelper.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/InvProtocol.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/InvQueue.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/InvScheduler.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/Message.c"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/RingByteBuffer.c"
)

# Add include paths
target_include_directories(${stm32F756_ProjectName} PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}/Inc"
	"${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include"
	"${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/Include"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
	"${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
	"${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App"
	"${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target"

	# eMD SmartMotion Driver
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Ak0991x"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils"
	"$ENV{INVN_EMD_SDK_PATH}/sources/Invn"
)

# Target compiler definitions
target_compile_definitions(${stm32F756_ProjectName} PRIVATE STM32F756xx)

# Add linked libraries
find_package(scpi CONFIG REQUIRED)
target_link_libraries(${stm32F756_ProjectName} PRIVATE manisonik::scpi)

find_package(fusion CONFIG REQUIRED)
target_link_libraries(${stm32F756_ProjectName} PRIVATE xioTechnologies::fusion)

target_link_libraries(${stm32F756_ProjectName} PRIVATE icm20948)
target_link_libraries(${stm32F756_ProjectName} PRIVATE dshot)
target_link_libraries(${stm32F756_ProjectName} PRIVATE icp10101)

# Compiler options
target_compile_options(${stm32F756_ProjectName} PRIVATE
    #-Wall
    #-Wextra
    #-Wpedantic
    #-Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:C>: >
    $<$<COMPILE_LANGUAGE:CXX>:

    # -Wno-volatile
    # -Wold-style-cast
    # -Wuseless-cast
    # -Wsuggest-override
    >
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>
)

# Core MCU flags, CPU type, instruction set and FPU setup
set(cpu_PARAMS 
   -mthumb

    # Other parameters
    # -mcpu, -mfloat, -mfloat-abi, ...
    -mcpu=cortex-m7
	-mfpu=fpv5-sp-d16
	-mfloat-abi=hard
)

# Linker options
target_link_options(${stm32F756_ProjectName} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f756zgtx_FLASH.ld
    ${cpu_PARAMS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -u _printf_float # STDIO float formatting support (remove if not used)
    #--specs=nosys.specs
	--specs=rdimon.specs -lc -lrdimon
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
    -Wl,--print-memory-usage
	-Wmissing-braces
)

# Execute post-build to print size, generate hex and bin
add_custom_command(TARGET ${stm32F756_ProjectName} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${stm32F756_ProjectName}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${stm32F756_ProjectName}> ${stm32F756_ProjectName}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${stm32F756_ProjectName}> ${stm32F756_ProjectName}.bin
)