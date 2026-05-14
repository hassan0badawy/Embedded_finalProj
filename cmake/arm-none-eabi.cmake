set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Set the toolchain prefix
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Compiler paths
find_program(C_COMPILER_PATH ${TOOLCHAIN_PREFIX}gcc)
if(NOT C_COMPILER_PATH)
    message(FATAL_ERROR "Could not find arm-none-eabi-gcc. Please ensure it is in your PATH.")
endif()

find_program(CXX_COMPILER_PATH ${TOOLCHAIN_PREFIX}g++)
find_program(OBJCOPY_PATH ${TOOLCHAIN_PREFIX}objcopy)
find_program(SIZE_PATH ${TOOLCHAIN_PREFIX}size)

set(CMAKE_C_COMPILER "C:/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe" CACHE INTERNAL "")
set(CMAKE_ASM_COMPILER "C:/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe" CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER "C:/arm-gnu-toolchain/bin/arm-none-eabi-g++.exe" CACHE INTERNAL "")
set(CMAKE_OBJCOPY "C:/arm-gnu-toolchain/bin/arm-none-eabi-objcopy.exe" CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER ${CXX_COMPILER_PATH})
set(CMAKE_OBJCOPY ${OBJCOPY_PATH} CACHE INTERNAL "")
set(CMAKE_SIZE ${SIZE_PATH} CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Bypass compiler checks which require linking a full executable
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
