cmake_minimum_required(VERSION 3.22)

set(sensor_test_VERSION 1.0.0)

# Core project settings
project(sensor_test
    VERSION ${sensor_test_VERSION}
    DESCRIPTION "Generic Sensor Library"
    HOMEPAGE_URL "https://github.com/manisonik/sensor"
    LANGUAGES C CXX
)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
)

find_package(sensor)
target_link_libraries(${PROJECT_NAME} PRIVATE sensor)

find_package(GTest CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)

add_test(
    NAME ${PROJECT_NAME}
    COMMAND ${PROJECT_NAME} 
)
