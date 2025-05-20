cmake_minimum_required(VERSION 3.24)

# Toolchain
include("${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")

# Core project settings
set(sensor_VERSION 1.0.1)
project(sensor
    VERSION ${sensor_VERSION}
    DESCRIPTION "Generic Sensor Library"
    HOMEPAGE_URL "https://github.com/manisonik/sensor"
    LANGUAGES C ASM
)

# Setup compiler settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Core project settings
message("Build type: " ${CMAKE_BUILD_TYPE})

# if you haven't included it already
# definitions of CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_INCLUDEDIR and others
include(GNUInstallDirs)

# check if in the top-level project
if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS "This project has a top-level one called [${CMAKE_PROJECT_NAME}]")
else()
    message(STATUS "This project is a top-level one")
endif()

# add source directory
add_subdirectory(src)

# add tests directory
add_subdirectory(tests)

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

target_include_directories(dshot PUBLIC
        $<BUILD_INTERFACE:
            # dshot
			"${CMAKE_CURRENT_SOURCE_DIR}/include/dshot"
            >
        $<INSTALL_INTERFACE:"include/dshot">  # <prefix>/include/mylib
)

target_include_directories(icp10101 PUBLIC
        $<BUILD_INTERFACE:
            # icp10101
			"${CMAKE_CURRENT_SOURCE_DIR}/include/icp10101"
            >
        $<INSTALL_INTERFACE:"include/icp10101">  # <prefix>/include/mylib
)

target_include_directories(ibus PUBLIC
        $<BUILD_INTERFACE:
            # ibus
			"${CMAKE_CURRENT_SOURCE_DIR}/include/ibus"
            >
        $<INSTALL_INTERFACE:"include/ibus">  # <prefix>/include/mylib
)

target_include_directories(icm20948 PUBLIC
        $<BUILD_INTERFACE:
            # icm20948
			"${CMAKE_CURRENT_SOURCE_DIR}/include/icm20948"

            # eMD SmartMotion Driver
            "$ENV{INVN_EMD_SDK_PATH}/sources"
            "$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Ak0991x"
            "$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices/Drivers/Icm20948"
            "$ENV{INVN_EMD_SDK_PATH}/sources/Invn/Devices"
            "$ENV{INVN_EMD_SDK_PATH}/sources/Invn/DynamicProtocol"
            "$ENV{INVN_EMD_SDK_PATH}/sources/Invn/EmbUtils"
            "$ENV{INVN_EMD_SDK_PATH}/sources/Invn"
            >
        $<INSTALL_INTERFACE:"include/icm20948">  # <prefix>/include/mylib
)

# install the target and create export-set
install(TARGETS dshot
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
