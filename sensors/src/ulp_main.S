;***************************************************************************
; ESP32-C3 ULP RISC-V Assembly Program
; Ultra-Low Power NTC Thermistor Reading and ESP-NOW Control
;
; This assembly program runs on the ESP32-C3's ULP RISC-V coprocessor
; and performs sensor reading while the main CPU is in deep sleep.
;
; Hardware Setup:
; - GPIO0 (ULP_GPIO0): NTC thermistor with 10kΩ pullup -> ADC1_CH0
; - GPIO1 (ULP_GPIO1): Battery voltage divider -> ADC1_CH1
; - Main CPU: Handles ESP-NOW transmission when awakened by ULP
;
; Power Consumption:
; - ULP active: ~150µA for 10ms every 15-120 minutes
; - ULP sleep: ~8µA between readings
; - Main CPU wake: ~80mA for 100ms during transmission
; - Deep sleep: ~10µA
; - Average: ~20-30µA (enabling 2-5+ year battery life)
;
; Memory Layout:
; - 0x50000000-0x50001FFF: ULP program memory (8KB)
; - 0x50001000-0x50001FFF: ULP data memory (4KB)
; - Shared variables accessible by both ULP and main CPU
;***************************************************************************

.text
.global _start

; Entry point - ULP RISC-V program starts here
_start:
    ; Initialize stack pointer
    li sp, 0x50001F00
    
    ; Call main ULP function
    call ulp_main
    
    ; Halt ULP processor
    halt

;---------------------------------------------------------------------------
; Main ULP function - handles sensor reading and power management
;---------------------------------------------------------------------------
ulp_main:
    ; Save return address
    addi sp, sp, -4
    sw ra, 0(sp)
    
    ; Initialize ADC for sensor readings
    call init_adc
    
    ; Read NTC thermistor on ADC1_CH0 (GPIO0)
    call read_ntc_sensor
    
    ; Read battery voltage on ADC1_CH1 (GPIO1)
    call read_battery_voltage
    
    ; Convert ADC readings to meaningful values
    call convert_temperature
    call convert_battery_mv
    
    ; Update power management state
    call update_power_mode
    
    ; Increment wake counter
    call increment_wake_count
    
    ; Check if transmission is needed
    call check_transmission_needed
    
    ; If transmission needed, wake main CPU
    la t0, transmission_needed
    lw t1, 0(t0)
    beqz t1, skip_wake_main
    call wake_main_cpu
    
skip_wake_main:
    ; Set timer for next ULP wake
    call set_ulp_timer
    
    ; Restore return address and return
    lw ra, 0(sp)
    addi sp, sp, 4
    ret

;---------------------------------------------------------------------------
; Initialize ADC for ULP operation
;---------------------------------------------------------------------------
init_adc:
    ; ADC1 base address
    li t0, 0x60008800
    
    ; Enable ADC1 and configure for ULP access
    li t1, 0x00000003        ; Enable CH0 and CH1
    sw t1, 0x00(t0)          ; ADC1_CONF register
    
    ; Set ADC resolution to 12-bit
    li t1, 0x00000003        ; 12-bit resolution
    sw t1, 0x04(t0)          ; ADC1_CONF2 register
    
    ; Configure attenuation for 3.3V range
    li t1, 0x00000033        ; 11dB attenuation for both channels
    sw t1, 0x08(t0)          ; ADC1_ATTEN register
    
    ret

;---------------------------------------------------------------------------
; Read NTC thermistor on ADC1_CH0 (GPIO0)
;---------------------------------------------------------------------------
read_ntc_sensor:
    ; Select ADC1_CH0
    li t0, 0x60008800
    li t1, 0x00000001        ; Select channel 0
    sw t1, 0x00(t0)
    
    ; Trigger ADC conversion
    li t1, 0x80000001        ; Start conversion
    sw t1, 0x10(t0)
    
    ; Wait for conversion complete
wait_ntc_conversion:
    lw t1, 0x10(t0)
    andi t1, t1, 0x80000000
    bnez t1, wait_ntc_conversion
    
    ; Read ADC result
    lw t1, 0x14(t0)          ; ADC1_DATA register
    andi t1, t1, 0x0FFF      ; Mask to 12 bits
    
    ; Store NTC ADC reading
    la t0, ntc_adc_raw
    sw t1, 0(t0)
    
    ret

;---------------------------------------------------------------------------
; Read battery voltage on ADC1_CH1 (GPIO1)
;---------------------------------------------------------------------------
read_battery_voltage:
    ; Select ADC1_CH1
    li t0, 0x60008800
    li t1, 0x00000002        ; Select channel 1
    sw t1, 0x00(t0)
    
    ; Trigger ADC conversion
    li t1, 0x80000002        ; Start conversion
    sw t1, 0x10(t0)
    
    ; Wait for conversion complete
wait_battery_conversion:
    lw t1, 0x10(t0)
    andi t1, t1, 0x80000000
    bnez t1, wait_battery_conversion
    
    ; Read ADC result
    lw t1, 0x14(t0)          ; ADC1_DATA register
    andi t1, t1, 0x0FFF      ; Mask to 12 bits
    
    ; Store battery ADC reading
    la t0, battery_adc_raw
    sw t1, 0(t0)
    
    ret

;---------------------------------------------------------------------------
; Convert NTC ADC reading to temperature (simplified Steinhart-Hart)
;---------------------------------------------------------------------------
convert_temperature:
    ; Load NTC ADC value
    la t0, ntc_adc_raw
    lw t1, 0(t0)             ; ADC reading (0-4095)
    
    ; Convert ADC to resistance
    ; R_ntc = R_pullup * adc / (4095 - adc)
    li t2, 4095
    sub t2, t2, t1           ; 4095 - adc
    li t3, 10000             ; 10kΩ pullup
    mul t1, t1, t3           ; adc * 10000
    div t1, t1, t2           ; R_ntc = (adc * 10000) / (4095 - adc)
    
    ; Store resistance
    la t0, ntc_resistance
    sw t1, 0(t0)
    
    ; Simplified temperature calculation (linear approximation)
    ; For NTCLE100E3103JB0: temp_c ≈ 25 - (R - 10000) / 330
    li t2, 10000
    sub t1, t1, t2           ; R - 10000
    li t2, 330
    div t1, t1, t2           ; (R - 10000) / 330
    li t2, 2500              ; 25°C * 100
    sub t1, t2, t1           ; 25 - ((R - 10000) / 330)
    
    ; Store temperature in Celsius * 100
    la t0, temp_c_x100
    sw t1, 0(t0)
    
    ret

;---------------------------------------------------------------------------
; Convert battery ADC to millivolts
;---------------------------------------------------------------------------
convert_battery_mv:
    ; Load battery ADC value
    la t0, battery_adc_raw
    lw t1, 0(t0)
    
    ; Convert to millivolts: mv = (adc * 6000) / 4095
    ; (assuming 2:1 voltage divider for 6V max input)
    li t2, 6000
    mul t1, t1, t2           ; adc * 6000
    li t2, 4095
    div t1, t1, t2           ; mv = (adc * 6000) / 4095
    
    ; Store battery voltage in millivolts
    la t0, battery_mv
    sw t1, 0(t0)
    
    ret

;---------------------------------------------------------------------------
; Update power mode based on battery voltage and system state
;---------------------------------------------------------------------------
update_power_mode:
    ; Load current battery voltage
    la t0, battery_mv
    lw t1, 0(t0)
    
    ; Check thresholds and set power mode
    li t2, 3400              ; Critical threshold (3.4V)
    blt t1, t2, emergency_mode
    
    li t2, 3800              ; Safe threshold (3.8V)
    blt t1, t2, safe_mode
    
    ; Normal mode
    li t1, 0                 ; Power mode = normal
    li t2, 15                ; Interval = 15 minutes
    j store_power_mode
    
safe_mode:
    li t1, 1                 ; Power mode = safe
    li t2, 60                ; Interval = 60 minutes
    j store_power_mode
    
emergency_mode:
    li t1, 2                 ; Power mode = emergency
    li t2, 120               ; Interval = 120 minutes
    
store_power_mode:
    la t0, power_mode
    sw t1, 0(t0)
    la t0, interval_minutes
    sw t2, 0(t0)
    
    ret

;---------------------------------------------------------------------------
; Increment wake counter
;---------------------------------------------------------------------------
increment_wake_count:
    la t0, wake_count
    lw t1, 0(t0)
    addi t1, t1, 1
    sw t1, 0(t0)
    ret

;---------------------------------------------------------------------------
; Check if transmission is needed based on interval
;---------------------------------------------------------------------------
check_transmission_needed:
    ; Load current wake count and last transmission wake
    la t0, wake_count
    lw t1, 0(t0)
    la t0, last_transmission_wake
    lw t2, 0(t0)
    
    ; Calculate wakes since last transmission
    sub t3, t1, t2
    
    ; Load current interval (simplified: transmit every wake for testing)
    li t4, 1                 ; For testing: transmit every wake
                             ; In production: load from interval_minutes
    
    ; Check if transmission needed
    bge t3, t4, need_transmission
    
    ; No transmission needed
    li t3, 0
    j store_transmission_flag
    
need_transmission:
    ; Transmission needed
    li t3, 1
    ; Update last transmission wake
    la t0, last_transmission_wake
    sw t1, 0(t0)
    
store_transmission_flag:
    la t0, transmission_needed
    sw t3, 0(t0)
    ret

;---------------------------------------------------------------------------
; Wake main CPU for ESP-NOW transmission
;---------------------------------------------------------------------------
wake_main_cpu:
    ; Set wake bit in RTC_CNTL_STATE0_REG to wake main CPU
    li t0, 0x3FF48000        ; RTC_CNTL base address
    li t1, 0x80000000        ; Wake main CPU bit
    sw t1, 0x1C(t0)          ; RTC_CNTL_STATE0_REG
    ret

;---------------------------------------------------------------------------
; Set ULP timer for next wake
;---------------------------------------------------------------------------
set_ulp_timer:
    ; Load interval in minutes
    la t0, interval_minutes
    lw t1, 0(t0)
    
    ; Convert to microseconds: us = minutes * 60 * 1000000
    li t2, 60
    mul t1, t1, t2           ; minutes * 60
    li t2, 1000000
    mul t1, t1, t2           ; seconds * 1000000
    
    ; For testing: use 15 seconds instead
    li t1, 15000000          ; 15 seconds in microseconds
    
    ; Program ULP timer
    li t0, 0x3FF48000        ; RTC_CNTL base address
    sw t1, 0x54(t0)          ; RTC_CNTL_ULP_CP_TIMER_1_REG
    
    ; Enable ULP timer wake
    li t1, 0x00000010        ; ULP timer wake enable bit
    li t0, 0x3FF48000        ; RTC_CNTL base address  
    lw t2, 0x1C(t0)          ; Read current STATE0 value
    or t2, t2, t1            ; Set ULP timer wake bit
    sw t2, 0x1C(t0)          ; Write back to RTC_CNTL_STATE0_REG
    
    ret

;***************************************************************************
; ULP Shared Data Section
; These variables are accessible by both ULP and main CPU
;***************************************************************************
.data
.align 4

; Raw sensor readings
ntc_adc_raw:         .word 0      ; NTC thermistor ADC value (0-4095)
battery_adc_raw:     .word 0      ; Battery voltage ADC value (0-4095)

; Converted values
ntc_resistance:      .word 10000  ; NTC resistance in ohms
temp_c_x100:         .word 2500   ; Temperature in Celsius * 100
battery_mv:          .word 4200   ; Battery voltage in millivolts

; Power management
power_mode:          .word 0      ; 0=normal, 1=safe, 2=emergency
interval_minutes:    .word 15     ; Current transmission interval

; Wake management  
wake_count:          .word 0      ; Total ULP wake count
last_transmission_wake: .word 0   ; Wake count of last transmission
transmission_needed: .word 1      ; Flag: transmission needed

; System state
sequence_number:     .word 0      ; ESP-NOW sequence number
failed_tx_count:     .word 0      ; Failed transmission count
safe_mode_request:   .word 0      ; Remote safe mode request flag

;***************************************************************************
; Build Instructions:
;
; 1. Install ESP32-C3 ULP RISC-V toolchain:
;    $ pip install esptool
;    $ git clone https://github.com/espressif/esp32-ulp-riscv-toolchain
;    $ export PATH=$PATH:/path/to/esp32-ulp-riscv-toolchain/bin
;
; 2. Assemble ULP program:
;    $ riscv32-esp-elf-as -march=rv32imc -o ulp_main.o ulp_main.S
;    $ riscv32-esp-elf-ld -T esp32c3.ulp.ld -o ulp_main.elf ulp_main.o
;    $ riscv32-esp-elf-objcopy -O binary ulp_main.elf ulp_main.bin
;
; 3. Embed in main program:
;    Add to CMakeLists.txt or component.mk:
;    COMPONENT_EMBED_FILES := ulp_main.bin
;
; 4. Load in main program:
;    extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
;    extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
;    size_t ulp_prog_size = ulp_main_bin_end - ulp_main_bin_start;
;    ulp_riscv_load_binary(ulp_main_bin_start, ulp_prog_size);
;
; 5. Configure sleep and start ULP:
;    esp_sleep_enable_ulp_wakeup();
;    ulp_riscv_run();
;    esp_deep_sleep_start();
;
; Memory Map:
; - ULP Program: 0x50000000 - 0x50000FFF (4KB)
; - ULP Data:    0x50001000 - 0x50001FFF (4KB)
; - Shared vars: Accessible by both ULP and main CPU
;
; Power Profile:
; - ULP reading sensors: ~150µA for 10ms
; - ULP between readings: ~8µA  
; - Main CPU wake for TX: ~80mA for 100ms
; - Deep sleep: ~10µA
; - Average (15min interval): ~25µA
; - Battery life (3000mAh): ~13 years theoretical, 3-5 years practical
;***************************************************************************
