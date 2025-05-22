# Common files
set(CC1200_COMMON_SOURCES
  common/CC1200Bits.h)

# STM32 HAL implementation
set(CC1200_STM32_SOURCES
  stm32/CC1200_HAL.h
  stm32/CC1200_HAL.cpp)

# MBED implementation
set(CC1200_MBED_SOURCES
  mbed/CC1200.h
  mbed/CC1200.cpp)

# All sources for the STM32 HAL implementation
set(CC1200_SOURCES
  ${CC1200_STM32_SOURCES}
  ${CC1200_COMMON_SOURCES})

# Use these CMake options to turn debug prints on or off.
option(CC1200_DEBUG "If true, include debug strings into the code." TRUE)
option(CC1200_REGISTER_LEVEL_DEBUG "If true, print debug messages for individual register writes" FALSE)

# STM32 HAL Configuration
# These paths should be adjusted based on your STM32 HAL installation
set(STM32_HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../STM32CubeF4)
set(STM32_HAL_INCLUDE_DIRS
  ${STM32_HAL_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  ${STM32_HAL_DIR}/Drivers/CMSIS/Include
  ${STM32_HAL_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
)

add_library(cc1200 ${CC1200_SOURCES})
target_include_directories(cc1200 PUBLIC . stm32 common ${STM32_HAL_INCLUDE_DIRS})

# Link against STM32 HAL libraries
target_link_libraries(cc1200 stm32f4xx_hal)

# Prevent gcc complaining about unknown IDE pragmas
target_compile_options(cc1200 PRIVATE -Wno-unknown-pragmas)

# Apply debug definitions.
# These are only used in CC1200_HAL.cpp so can be set as PRIVATE.
if(CC1200_DEBUG)
    target_compile_definitions(cc1200 PRIVATE CC1200_DEBUG)
endif()
if(CC1200_REGISTER_LEVEL_DEBUG)
    target_compile_definitions(cc1200 PRIVATE CC1200_REGISTER_LEVEL_DEBUG)
endif()

# Add the original MBED-OS version as a separate target for reference
add_library(cc1200_mbed EXCLUDE_FROM_ALL ${CC1200_MBED_SOURCES} ${CC1200_COMMON_SOURCES})
target_include_directories(cc1200_mbed PUBLIC . mbed common)
# Note: This target won't build without mbed-os, but is kept for reference

# Morse code example
set(CC1200_MORSE_SOURCES
  examples/morse/CC1200Morse.h
  examples/morse/CC1200Morse.cpp)

add_library(cc1200_morse EXCLUDE_FROM_ALL ${CC1200_MORSE_SOURCES} ${CC1200_MBED_SOURCES} ${CC1200_COMMON_SOURCES})
target_include_directories(cc1200_morse PUBLIC . mbed common examples/morse)
# Note: This target won't build without mbed-os, but is kept for reference