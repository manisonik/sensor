cmake_minimum_required(VERSION 3.22)

set(sensor_VERSION 1.0.0)

# Core project settings
project(sensor
    VERSION ${sensor_VERSION}
    DESCRIPTION "Generic Sensor Library"
    HOMEPAGE_URL "https://github.com/manisonik/sensor"
    LANGUAGES C
)

# Setup compiler settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

enable_testing()
add_subdirectory(tests)

# if you haven't included it already
# definitions of CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_INCLUDEDIR and others
include(GNUInstallDirs)

# check if in the top-level project
if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS "This project has a top-level one called [${CMAKE_PROJECT_NAME}]")
else()
    message(STATUS "This project is a top-level one")
endif()

# create the shared library (DLL)
add_library(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor.c

    # eMD SmartMotion Driver
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Ak0991x/Ak0991x.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Setup.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948/Icm20948Transport.c
	${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Sensor.c
	${INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol/DynProtocol.c
	${INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol/DynProtocolTransportUart.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/DataConverter.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/ErrorHelper.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/InvProtocol.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/InvQueue.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/InvScheduler.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/Message.c
	${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils/RingByteBuffer.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # eMD SmartMotion Driver
        ${INVN_EMD_SDK_PATH}/sources
        ${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Ak0991x
        ${INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948
        ${INVN_EMD_SDK_PATH}/sources/Invn/Devices
        ${INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol
        ${INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils
        ${INVN_EMD_SDK_PATH}/sources/Invn
        >
    $<INSTALL_INTERFACE:include/sensor>  # <prefix>/include/mylib    
)

# note that it is not CMAKE_INSTALL_PREFIX we are checking here
if(DEFINED CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    message(
        STATUS
        "CMAKE_INSTALL_PREFIX is not set\n"
        "Default value: ${CMAKE_INSTALL_PREFIX}\n"
        "Will set it to ${CMAKE_SOURCE_DIR}/install"
    )
    set(CMAKE_INSTALL_PREFIX
        "${CMAKE_SOURCE_DIR}/install"
        CACHE PATH "Where the library will be installed to" FORCE
    )
else()
    message(
        STATUS
        "CMAKE_INSTALL_PREFIX was already set\n"
        "Current value: ${CMAKE_INSTALL_PREFIX}"
    )
endif()

# without it public headers won't get installed
set(public_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensor.h
)

# note that ${public_headers} has to be in quotes
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${public_headers}")

# set the debug post-fix
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")

# install the target and create export-set
install(TARGETS ${PROJECT_NAME}
    EXPORT "${PROJECT_NAME}Targets"
    # these get default values from GNUInstallDirs, no need to set them
    #RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} # bin
    #LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} # lib
    #ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # lib
    # except for public headers, as we want them to be inside a library folder
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} # include/SomeLibrary
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

# generate and install export file
install(EXPORT "${PROJECT_NAME}Targets"
    FILE "${PROJECT_NAME}Targets.cmake"
    NAMESPACE manisonik::
    DESTINATION cmake
)

include(CMakePackageConfigHelpers)

# generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${version}"
    COMPATIBILITY AnyNewerVersion
)
# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION cmake
)
# install config files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION cmake
)

