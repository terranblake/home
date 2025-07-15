; ULP RISC-V assembly for simplified coin cell battery test
; This file must be assembled separately and loaded into ULP memory

.section .text
.global entry
entry:
    ; Read thermistor ADC (ADC1_CH0)
    move r0, 0                       ; ADC1_CHANNEL_0
    adc r1, r0, 0                    ; Read ADC, store in r1
    
    ; Store thermistor ADC value in RTC memory
    ; Assuming adc_reading is at RTC_SLOW_MEM[0]
    st r1, r0, 0                     ; Store value
    
    ; Read battery ADC (ADC1_CH1)
    move r0, 1                       ; ADC1_CHANNEL_1
    adc r1, r0, 0                    ; Read ADC, store in r1
    
    ; Store battery ADC value in RTC memory
    ; Assuming battery_reading is at RTC_SLOW_MEM[1]
    st r1, r0, 1                     ; Store value
    
    ; Wake up the main CPU
    wake                              ; Wake up main CPU
    
    halt                              ; Halt ULP program until next timer event
