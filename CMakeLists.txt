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
set(CMAKE_PROJECT_NAME stm32h7-ethcan-bridge)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Fetch CANopenNode_STM32
include(FetchContent)
FetchContent_Declare(
    canopen_stm32
    GIT_REPOSITORY https://github.com/CANopenNode/CanOpenSTM32.git
    GIT_TAG master
)
FetchContent_MakeAvailable(canopen_stm32)

# --- Minimal fixes: ensure we get the populated source dir and initialize submodules ---
FetchContent_GetProperties(canopen_stm32)
if(NOT canopen_stm32_POPULATED)
  message(FATAL_ERROR "canopen_stm32 was not populated by FetchContent.")
endif()

# set a stable variable used later in the file
set(CANOPEN_STM32_DIR ${canopen_stm32_SOURCE_DIR})

# If the downloaded repo contains git metadata, try to init/update submodules
find_package(Git QUIET)
if(GIT_EXECUTABLE AND EXISTS "${CANOPEN_STM32_DIR}/.git")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
    WORKING_DIRECTORY ${CANOPEN_STM32_DIR}
    RESULT_VARIABLE _git_submod_result
    OUTPUT_QUIET ERROR_QUIET
  )
  if(NOT _git_submod_result EQUAL 0)
    message(WARNING "git submodule update returned ${_git_submod_result}; submodules may be missing")
  endif()
endif()
# -------------------------------------------------------------------------------

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/User/print_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/mqtt_client.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/can_ctrl.c
    # CANopen STM32 Driver and Stack
    ${CANOPEN_STM32_DIR}/CANopenNode_STM32/CO_driver_STM32.c
    ${CANOPEN_STM32_DIR}/CANopenNode_STM32/CO_app_STM32.c
    ${CANOPEN_STM32_DIR}/CANopenNode/CANopen.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_NMT_Heartbeat.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_HBconsumer.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_Emergency.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_SDOserver.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_SDOclient.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_SYNC.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_PDO.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_fifo.c
    ${CANOPEN_STM32_DIR}/CANopenNode/303/CO_LEDs.c
    ${CANOPEN_STM32_DIR}/CANopenNode/304/CO_GFC.c
    ${CANOPEN_STM32_DIR}/CANopenNode/305/CO_LSSslave.c
    ${CANOPEN_STM32_DIR}/CANopenNode/305/CO_LSSmaster.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_ODinterface.c
    ${CANOPEN_STM32_DIR}/CANopenNode/301/CO_TIME.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User includes
    ${CMAKE_CURRENT_SOURCE_DIR}/User
    
    # CANopen includes
    ${CANOPEN_STM32_DIR}/CANopenNode
    ${CANOPEN_STM32_DIR}/CANopenNode/301
    ${CANOPEN_STM32_DIR}/CANopenNode/303
    ${CANOPEN_STM32_DIR}/CANopenNode/304
    ${CANOPEN_STM32_DIR}/CANopenNode/305
    ${CANOPEN_STM32_DIR}/CANopenNode_STM32
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USE_HAL_DRIVER
    STM32H743xx
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
