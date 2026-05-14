set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/arm-none-eabi.cmake")
project(Embedded_finalProj C ASM)


# Toolchain check
if(NOT CMAKE_TOOLCHAIN_FILE)
    message(WARNING "No toolchain file specified. Defaulting to host compiler. "
                    "For STM32, use: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake ..")
endif()

# Set C Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Include Directories
include_directories(
    Core/Inc
)

# Assembler Include Directories
include_directories("startup")

# Source Files
file(GLOB_RECURSE SOURCES
    "Core/Src/*.c"
)

# Startup File
set(ASM_SOURCES
    "startup/STM32F4xx/STM32F401xE.s"
)

# Executables
set(TARGETS ${PROJECT_NAME}_Master.elf ${PROJECT_NAME}_Slave.elf)

foreach(TARGET_NAME ${TARGETS})
    add_executable(${TARGET_NAME} ${SOURCES} ${ASM_SOURCES})

    if(TRUE)
        target_compile_options(${TARGET_NAME} PRIVATE
            -mcpu=cortex-m4
            -mthumb
            -mfpu=fpv4-sp-d16
            -mfloat-abi=hard
            -Wall
            -Wextra
            -Wpedantic
            -fdata-sections
            -ffunction-sections
            -g
        )

        # Remove the .elf extension for the map file name to match common conventions (optional but clean)
        string(REPLACE ".elf" ".map" MAP_NAME ${TARGET_NAME})

        target_link_options(${TARGET_NAME} PRIVATE
            -mcpu=cortex-m4
            -mthumb
            -mfpu=fpv4-sp-d16
            -mfloat-abi=hard
            -specs=nano.specs
            -Wl,--gc-sections
            -Wl,-Map=${MAP_NAME}
            -T ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F4xx/STM32F401xE.ld
            -L ${CMAKE_CURRENT_SOURCE_DIR}/linker
        )
    endif()

    # Post-build: Create hex and bin files
    if(CMAKE_OBJCOPY)
        string(REPLACE ".elf" ".hex" HEX_NAME ${TARGET_NAME})
        string(REPLACE ".elf" ".bin" BIN_NAME ${TARGET_NAME})
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET_NAME} ${HEX_NAME}
            COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_NAME} ${BIN_NAME}
            COMMENT "Generating Hex and Binary files for ${TARGET_NAME}"
        )
    endif()
endforeach()

# Set Master/Slave definitions
target_compile_definitions(${PROJECT_NAME}_Master.elf PRIVATE IS_MASTER=1)
target_compile_definitions(${PROJECT_NAME}_Slave.elf PRIVATE IS_MASTER=0)
