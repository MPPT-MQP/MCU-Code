# FinalMCUCode
### Code Written by: MPPT MQP 2024-2025 (Kyle, Frank, Micaela, Saketh)

#### Code is designed for the RP2350. It should also work with the RP2040 (Pico1) but will need slight adjustments due to the picoSDK.
#### To run the code, you will need the picoSDK and CMake, which is easily installed using the Pico VSCode extension but manual installs will also work

____
### List of Files:

| File Name | Details |
| ------------- | ------------- |
| algorithms.c / .h    | Contains all MPPT algorithms and PID controller  |
| buttons.c / .h       | Functions that initialize buttons and setup ISR & debouncing  |
| CMakeLists.txt       | CMake Configuration File |
| def.h                | Configuration file to select algorithm, enable/disable OLED screen, and DC-DC converter frequency |
| hw_config.h          | Config file from no-OS-FatFS SD card library |
| MCUCode.c            | Main code file for both core0 and core1 |
| oled_screen.c / .h   | Functions to interface with the SSD1306 display IC|
| PowerMonitor.h       | Defines for the INA740B power monitor |
| sdCard.c / .h        | Functions to init Sd card, mount, and write data to the buffer & card |
| sensors.c / .h       | Functions to configure I2C and interface with PowerMonitors, Pyranometer, Temperature Sensor (ADC), PWM Generator, ExternalADC, and RTC |
| TMP102.h             | Defines for the TMP102 temperature sensor |
| user_interface.c/.h  | User interface for the OLED screen. View sensor data, change algorithm, and other settings using buttons|
| Lib/no-OS-FatFS      | Library files for the no-OS-FatFS Pico library. Ported by Carl J Kugler III from FatFS |
| Lib/PID              | Library files for the PID controller library. Ported by Samyar Sadat Akhavi, originally written by Brett Beauregard | 
