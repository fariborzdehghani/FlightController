cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FlightController)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Core.c starts the watchdog after sensor calibration, rather than in the
# generated startup sequence. Keep its HAL driver enabled across regeneration.
target_compile_definitions(stm32cubemx INTERFACE HAL_IWDG_MODULE_ENABLED)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/Core.c
    Core/Src/Tools.c
    Core/Src/FlightControl.c
    Core/Modules/Src/AppError.c
    Core/Modules/Src/CoreHelpers.c
    Core/Modules/Src/SensorAcquisition.c
    Core/Modules/Src/VelocityEstimator.c
    Core/Src/MPU6050.c
    Core/Src/MotorControl.c
    Core/Src/sx127x.c
    Core/Src/PID.c
    Core/Src/srf05.c
    Core/Src/QMC5883.c
    Core/Src/BMPXX80.c
    Core/Src/Tests.c
    Core/Src/Logger.c
    Core/Src/FlightRecorder.c
    Core/Src/Calibration.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Modules/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Identify the estimator sources in downloaded traces, including uncommitted
# edits. Filter overrides are recorded separately in the log header.
set(imu_source_hashes "")
foreach(imu_source Core/Src/MPU6050.c Core/Inc/MPU6050.h
                   Core/Inc/AttitudeTrace.h Core/Inc/SensorConfig.h)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${imu_source}")
    file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${imu_source}" imu_source_hash)
    string(APPEND imu_source_hashes "${imu_source_hash}")
endforeach()
string(SHA256 imu_signature "${imu_source_hashes}")
string(SUBSTRING "${imu_signature}" 0 8 imu_signature)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    FLIGHT_LOG_ESTIMATOR_ID=0x${imu_signature}u)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Ensure printf supports floating-point on newlib (enable %f)
# Adds the linker undefined symbol to pull in float support from newlib
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-u" "_printf_float")
endif()
