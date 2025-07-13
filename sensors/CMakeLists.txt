# CMakeLists.txt for ESP32-C3 ULP RISC-V Program
# This file enables building the ULP assembly program and embedding it in the main binary

cmake_minimum_required(VERSION 3.16)

# Set component name
set(COMPONENT_NAME "esp32_ulp_temperature_sensor")

# Add ULP RISC-V support
set(CONFIG_ESP32C3_ULP_RISCV_ENABLED y)
set(CONFIG_ULP_COPROC_ENABLED y)
set(CONFIG_ULP_COPROC_TYPE_RISCV y)

# Define ULP source files
set(ULP_S_SOURCES "ulp_main.S")
set(ULP_EXP_DEP_SRCS "ulp_receiver.cpp")

# ULP RISC-V toolchain configuration
if(CONFIG_ESP32C3_ULP_RISCV_ENABLED)
    # Add ULP component
    set(ULP_RISCV_TOOLCHAIN_PREFIX "riscv32-esp-elf-")
    
    # ULP RISC-V assembler flags
    set(ULP_AS_FLAGS 
        -march=rv32imc
        -mabi=ilp32
        -c
        -g
        -Wall
        -Werror
        -nostdlib
        -nostartfiles
        -fno-builtin
        -fdata-sections
        -ffunction-sections
    )
    
    # ULP RISC-V linker flags  
    set(ULP_LD_FLAGS
        -T${IDF_PATH}/components/ulp/ld/esp32c3.ulp.ld
        -nostdlib
        -nostartfiles
        --gc-sections
        -Map=ulp_main.map
    )
    
    # Custom build rules for ULP program
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.o
        COMMAND ${ULP_RISCV_TOOLCHAIN_PREFIX}as ${ULP_AS_FLAGS} 
                -o ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.o 
                ${CMAKE_CURRENT_SOURCE_DIR}/ulp_main.S
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ulp_main.S
        COMMENT "Assembling ULP RISC-V program"
    )
    
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.elf
        COMMAND ${ULP_RISCV_TOOLCHAIN_PREFIX}ld ${ULP_LD_FLAGS}
                -o ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.elf
                ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.o
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.o
        COMMENT "Linking ULP RISC-V program"
    )
    
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.bin
        COMMAND ${ULP_RISCV_TOOLCHAIN_PREFIX}objcopy -O binary
                ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.elf
                ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.bin
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.elf
        COMMENT "Creating ULP RISC-V binary"
    )
    
    # Create custom target for ULP build
    add_custom_target(ulp_main_target
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.bin
    )
    
    # Embed ULP binary in main program
    target_add_binary_data(${COMPONENT_LIB} 
        ${CMAKE_CURRENT_BINARY_DIR}/ulp_main.bin 
        BINARY
    )
    
    # Add dependency
    add_dependencies(${COMPONENT_LIB} ulp_main_target)
    
    # Export ULP symbols for main program
    set_property(TARGET ${COMPONENT_LIB} 
        APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
        ULP_RISCV_BINARY_EMBEDDED=1
    )
    
    message(STATUS "ULP RISC-V support enabled for ESP32-C3")
else()
    message(WARNING "ULP RISC-V not enabled - using simulated ULP operation")
endif()

# Component registration
idf_component_register(
    SRCS "ulp_receiver.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        "esp_wifi"
        "esp_hw_support"
        "ulp"
        "driver"
        "esp_system"
        "nvs_flash"
        "esp_common"
        "freertos"
)

# Additional compile definitions for ULP receiver
target_compile_definitions(${COMPONENT_LIB} 
    PRIVATE
    -DESP32C3_ULP_RISCV_ENABLED=$<BOOL:${CONFIG_ESP32C3_ULP_RISCV_ENABLED}>
    -DULP_COPROC_ENABLED=$<BOOL:${CONFIG_ULP_COPROC_ENABLED}>
)

# Optimization flags for ultra-low power
if(CONFIG_COMPILER_OPTIMIZATION_SIZE)
    target_compile_options(${COMPONENT_LIB} 
        PRIVATE
        -Os              # Optimize for size
        -ffunction-sections
        -fdata-sections
    )
    
    target_link_options(${COMPONENT_LIB}
        PRIVATE  
        -Wl,--gc-sections  # Remove unused sections
    )
endif()

# Power management configuration
set_property(TARGET ${COMPONENT_LIB}
    APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
    CONFIG_PM_ENABLE=1
    CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND=y
    CONFIG_ESP_SLEEP_POWER_DOWN_FLASH=y
    CONFIG_ESP_SLEEP_RTC_FAST_MEM_KEEP_ISO=y
)

# Debug configuration
if(CONFIG_LOG_DEFAULT_LEVEL GREATER_EQUAL 3)
    target_compile_definitions(${COMPONENT_LIB} 
        PRIVATE
        ULP_DEBUG_ENABLED=1
    )
    message(STATUS "ULP debug logging enabled")
endif()

# Build information
message(STATUS "ESP32-C3 ULP Temperature Sensor Component Configuration:")
message(STATUS "  ULP RISC-V Enabled: ${CONFIG_ESP32C3_ULP_RISCV_ENABLED}")
message(STATUS "  ULP Coproc Enabled: ${CONFIG_ULP_COPROC_ENABLED}")
message(STATUS "  Source Files: ulp_receiver.cpp, ulp_main.S")
message(STATUS "  Target: Ultra-low power temperature sensing with ESP-NOW")
message(STATUS "  Expected Current: ~20-30µA average")
message(STATUS "  Expected Battery Life: 3-5+ years on 3x AA lithium")
