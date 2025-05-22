cmake_minimum_required(VERSION 3.10)
project(CC1200_Mock_Test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Common files
set(CC1200_COMMON_SOURCES
  ${CMAKE_SOURCE_DIR}/../common/CC1200Bits.h)

# STM32 HAL mock implementation
set(CC1200_STM32_MOCK_SOURCES
  ${CMAKE_SOURCE_DIR}/../mock/stm32/stm32f4xx_hal.h
  ${CMAKE_SOURCE_DIR}/../mock/stm32/stm32f4xx_hal.c)

# Mbed OS mock implementation
set(CC1200_MBED_MOCK_SOURCES
  ${CMAKE_SOURCE_DIR}/../mock/mbed/mbed.h
  ${CMAKE_SOURCE_DIR}/../mock/mbed/mbed.cpp)

# STM32 HAL driver implementation
set(CC1200_STM32_SOURCES
  ${CMAKE_SOURCE_DIR}/../stm32/CC1200_HAL.h
  ${CMAKE_SOURCE_DIR}/../stm32/CC1200_HAL.cpp)

# Mbed OS driver implementation
set(CC1200_MBED_SOURCES
  ${CMAKE_SOURCE_DIR}/../mbed/CC1200.h
  ${CMAKE_SOURCE_DIR}/../mbed/CC1200.cpp)

# Common sources
set(CC1200_COMMON_SOURCES
  ${CMAKE_SOURCE_DIR}/../common/CC1200Bits.h)

# Debug options
option(CC1200_DEBUG "If true, include debug strings into the code." TRUE)
option(CC1200_REGISTER_LEVEL_DEBUG "If true, print debug messages for individual register writes" FALSE)

# Create mock STM32 HAL library
add_library(stm32_hal_mock ${CC1200_STM32_MOCK_SOURCES})
target_include_directories(stm32_hal_mock PUBLIC ${CMAKE_SOURCE_DIR}/../mock/stm32)

# Create mock Mbed OS library
add_library(mbed_mock ${CC1200_MBED_MOCK_SOURCES})
target_include_directories(mbed_mock PUBLIC ${CMAKE_SOURCE_DIR}/../mock/mbed)

# Create CC1200 STM32 HAL driver library
add_library(cc1200_stm32 ${CC1200_STM32_SOURCES} ${CC1200_COMMON_SOURCES})
target_include_directories(cc1200_stm32 PUBLIC 
  ${CMAKE_SOURCE_DIR}/..
  ${CMAKE_SOURCE_DIR}/../stm32
  ${CMAKE_SOURCE_DIR}/../common
  ${CMAKE_SOURCE_DIR}/../mock/stm32)
target_link_libraries(cc1200_stm32 stm32_hal_mock)

# Create CC1200 Mbed OS driver library
add_library(cc1200_mbed ${CC1200_MBED_SOURCES} ${CC1200_COMMON_SOURCES})
target_include_directories(cc1200_mbed PUBLIC 
  ${CMAKE_SOURCE_DIR}/..
  ${CMAKE_SOURCE_DIR}/../mbed
  ${CMAKE_SOURCE_DIR}/../common
  ${CMAKE_SOURCE_DIR}/../mock/mbed)
target_link_libraries(cc1200_mbed mbed_mock)

# Apply debug definitions
if(CC1200_DEBUG)
    target_compile_definitions(cc1200_stm32 PRIVATE CC1200_DEBUG)
    target_compile_definitions(cc1200_mbed PRIVATE CC1200_DEBUG)
endif()
if(CC1200_REGISTER_LEVEL_DEBUG)
    target_compile_definitions(cc1200_stm32 PRIVATE CC1200_REGISTER_LEVEL_DEBUG)
    target_compile_definitions(cc1200_mbed PRIVATE CC1200_REGISTER_LEVEL_DEBUG)
endif()

# Create test application for STM32 HAL
add_executable(cc1200_stm32_test ${CMAKE_SOURCE_DIR}/../examples/mock_test.cpp)
target_compile_definitions(cc1200_stm32_test PRIVATE USE_STM32_HAL)
target_include_directories(cc1200_stm32_test PRIVATE 
  ${CMAKE_SOURCE_DIR}/..
  ${CMAKE_SOURCE_DIR}/../stm32
  ${CMAKE_SOURCE_DIR}/../common
  ${CMAKE_SOURCE_DIR}/../mock/stm32)
target_link_libraries(cc1200_stm32_test cc1200_stm32 stm32_hal_mock)

# Create test application for Mbed OS
add_executable(cc1200_mbed_test ${CMAKE_SOURCE_DIR}/../examples/mock_test.cpp)
target_include_directories(cc1200_mbed_test PRIVATE 
  ${CMAKE_SOURCE_DIR}/..
  ${CMAKE_SOURCE_DIR}/../mbed
  ${CMAKE_SOURCE_DIR}/../common
  ${CMAKE_SOURCE_DIR}/../mock/mbed)
target_link_libraries(cc1200_mbed_test cc1200_mbed mbed_mock)

# Add a custom target to run both tests
add_custom_target(run_tests
  COMMAND echo "Running STM32 HAL test..."
  COMMAND $<TARGET_FILE:cc1200_stm32_test>
  COMMAND echo "\nRunning Mbed OS test..."
  COMMAND $<TARGET_FILE:cc1200_mbed_test>
  DEPENDS cc1200_stm32_test cc1200_mbed_test
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Running CC1200 mock tests"
  VERBATIM
)
