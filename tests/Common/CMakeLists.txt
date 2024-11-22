set(sensor_PROJECT_NAME sensor_test)
add_executable(${sensor_PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_test.cc
)

find_package(GTest CONFIG REQUIRED)
target_link_libraries(${sensor_PROJECT_NAME} PRIVATE 
  dshot
  icm20948
  icp10101
  GTest::gtest
  GTest::gtest_main
  GTest::gmock
  GTest::gmock_main
)

gtest_discover_tests(${sensor_PROJECT_NAME})
