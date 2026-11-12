#******************************************************************************
# @file    gcc-arm-none-eabi.cmake
# @brief   CMake toolchain file: arm-none-eabi-gcc (GNU Tools for STM32)
#
#          工具链查找顺序：
#            1. 环境变量 STM32CUBECLT_PATH 指向的 ST 官方工具链
#            2. 默认安装路径 C:/ST/STM32CubeCLT_1.21.0
#            3. 系统 PATH 中的 arm-none-eabi-gcc
#******************************************************************************

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(TOOLCHAIN_PREFIX                arm-none-eabi-)

#---------------- 工具链路径 ----------------
if(DEFINED ENV{STM32CUBECLT_PATH})
    file(TO_CMAKE_PATH "$ENV{STM32CUBECLT_PATH}" TOOLCHAIN_ROOT)
elseif(EXISTS "C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")
    set(TOOLCHAIN_ROOT              "C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32")
endif()

if(TOOLCHAIN_ROOT)
    set(CMAKE_C_COMPILER            "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}gcc.exe")
    set(CMAKE_CXX_COMPILER          "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}g++.exe")
    set(CMAKE_ASM_COMPILER          "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}gcc.exe")
    set(CMAKE_OBJCOPY               "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}objcopy.exe")
    set(CMAKE_SIZE                  "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}size.exe")
else()
    set(CMAKE_C_COMPILER            ${TOOLCHAIN_PREFIX}gcc)
    set(CMAKE_CXX_COMPILER          ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_ASM_COMPILER          ${TOOLCHAIN_PREFIX}gcc)
    set(CMAKE_OBJCOPY               ${TOOLCHAIN_PREFIX}objcopy)
    set(CMAKE_SIZE                  ${TOOLCHAIN_PREFIX}size)
endif()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)
