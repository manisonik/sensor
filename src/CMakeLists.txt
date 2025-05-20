set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# create the shared library (DLL)
set(icp10101_NAME icp10101)
add_library(${icp10101_NAME}
	sensor_icp10101.c

	# Adding headers so IDEs can see them.
	${PROJECT_SOURCE_DIR}/include/icp10101/sensor_icp10101.h
)

target_include_directories(${icp10101_NAME} PRIVATE
	${PROJECT_SOURCE_DIR}/include/icp10101
)

# without it public headers won't get installed
set(icp10101_public_headers
    ${PROJECT_SOURCE_DIR}/include/icp10101/sensor_icp10101.h
)

# note that ${public_headers} has to be in quotes
set_target_properties(${icp10101_NAME} PROPERTIES PUBLIC_HEADER "${icp10101_public_headers}")

# set the debug post-fix
set_target_properties(${icp10101_NAME} PROPERTIES DEBUG_POSTFIX "d")

#============================================================================#

# create the shared library (DLL)
set(icm20948_NAME icm20948)
add_library(${icm20948_NAME}
	"sensor_icm20948.c"

	# Adding headers so IDEs can see them.
	"${PROJECT_SOURCE_DIR}/include/icm20948/sensor_icm20948.h"

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

target_compile_definitions(${icm20948_NAME} PRIVATE INV_MSG_ENABLE=INV_MSG_LEVEL_DEBUG)

target_include_directories(${icm20948_NAME} PRIVATE
	"${PROJECT_SOURCE_DIR}/include/icm20948"
)

# without it public headers won't get installed
set(icm20948_public_headers
    "${PROJECT_SOURCE_DIR}/include/icm20948/sensor_icm20948.h"
)

# note that ${public_headers} has to be in quotes
set_target_properties(${icm20948_NAME} PROPERTIES PUBLIC_HEADER "${icm20948_public_headers}")

# set the debug post-fix
set_target_properties(${icm20948_NAME} PROPERTIES DEBUG_POSTFIX "d")

#============================================================================#

# create the shared library (DLL)
set(sensor_NAME dshot)
add_library(${sensor_NAME}
    "protocol_dshot.c"

	# Adding headers so IDEs can see them.
	"${PROJECT_SOURCE_DIR}/include/dshot/protocol_dshot.h"
)

target_include_directories(${sensor_NAME} PRIVATE
	"${PROJECT_SOURCE_DIR}/include/dshot"
)

# without it public headers won't get installed
set(dshot_public_headers
    "${PROJECT_SOURCE_DIR}/include/dshot/protocol_dshot.h"
)

# note that ${public_headers} has to be in quotes
set_target_properties(${sensor_NAME} PROPERTIES PUBLIC_HEADER "${dshot_public_headers}")

# set the debug post-fix
set_target_properties(${sensor_NAME} PROPERTIES DEBUG_POSTFIX "d")

#============================================================================#

# create the shared library (DLL)
set(ibus_NAME ibus)
add_library(${ibus_NAME}
    "protocol_ibus.c"

	# Adding headers so IDEs can see them.
	"${PROJECT_SOURCE_DIR}/include/ibus/protocol_ibus.h"
)

target_include_directories(${ibus_NAME} PRIVATE
	"${PROJECT_SOURCE_DIR}/include/ibus"
)

# without it public headers won't get installed
set(ibus_public_headers
    "${PROJECT_SOURCE_DIR}/include/ibus/protocol_ibus.h"
)

# note that ${public_headers} has to be in quotes
set_target_properties(${ibus_NAME} PROPERTIES PUBLIC_HEADER "${ibus_public_headers}")

# set the debug post-fix
set_target_properties(${ibus_NAME} PROPERTIES DEBUG_POSTFIX "d")

target_compile_definitions(${ibus_NAME} PRIVATE USE_TELEMETRY_IBUS)
target_compile_definitions(${ibus_NAME} PRIVATE USE_TELEMETRY)

#============================================================================#

file(GLOB HEADER_FILES_SUB "${PROJECT_SOURCE_DIR}/include/dshot/*.h")
source_group("include" FILES ${HEADER_FILES_SUB})

file(GLOB HEADER_FILES_SUB "${PROJECT_SOURCE_DIR}/include/icm20948/*.h")
source_group("include" FILES ${HEADER_FILES_SUB})

file(GLOB HEADER_FILES_SUB "${PROJECT_SOURCE_DIR}/include/icp10101/*.h")
source_group("include" FILES ${HEADER_FILES_SUB})

file(GLOB HEADER_FILES_SUB "${PROJECT_SOURCE_DIR}/include/ibus/*.h")
source_group("include" FILES ${HEADER_FILES_SUB})

file(GLOB SOURCE_FILES_SUB "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
source_group("src" FILES ${SOURCE_FILES_SUB})