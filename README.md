# Software Sensor
- For hardware design see: https://github.com/larus-breeze/hw_sensor 

## Facts
- STM32CubeIDE
- STM32F407VG
- FreeRTOS
- IMU 
- GNSS 
- Static pressure 
- Differential pressure
- CAN interface
- Bluetooth ESP32
- RS232 NMEA output channels

# How to use it
Some usefull git commands to work with this repository and the included submodule **command line**: 

Clone via https:

    git clone --recurse-submodules https://github.com/larus-breeze/sw_sensor
    
Clone via ssh:

    git clone --recurse-submodules git@github.com:larus-breeze/sw_sensor.git

Switch to a branch:

    git checkout branch_name --recurse-submodules
      
Pull latest changes:

    git pull --recurse-submodules  
      
# STM32 Build Configurations
- Release used for release versions (Max. optimized, no debug info)
- Debug used for development

# Flash and prepare the sensor hardware
## STM32
- Flash via USB using the STM32CubeProgrammer and a compiled binary sw_sensor.elf file from here: https://github.com/larus-breeze/sw_sensor/releases  
Hold the Boot Button on power-on to start the STM32 in the DFU bootloader mode.
Use the STM32CubeProgrammer to flash the binary to the STM32 micro-controller.

## ESP32 controller
- Flash the *.ino file in ESP32_Firmware with arduino studio via the esp32 usb connector.
- Optionally: Use the arduino IDE to change the device name and the RF mode (Bluetooth or WLAN).

## Prepare an sd-card with configuration files
- put a larus_sensor_config.ini file (template in configuration/) in the sd cards root directory. Adjust the parameters as described in configuration/README.md
Initially the heading may be inaccurate as the magnetic calibration algorithm needs some time in the air to find the exact calibration. 
A 30 minutes flight with some right and left turns should be sufficient to calibrate the compass module. 
- Optionally: Create a directory with the name "logger" to enable logging of all measurement data with 100Hz via [.lrsx](https://github.com/larus-breeze/doc_larus/blob/master/documentation/LRSX_log_file_format.md) files. 
- Optionally: Create a directory with the name "eeprom" to enable the creation of cleartext eeprom dumps including the installed firmware version and hardware id. 


### Additional developer options
- Put an empty file with the name: "sensor.readings" in order to switch the serial output format from Larus NMEA syntax to pure sensor raw data values.

# Led signal indications
There are LEDs on the PCBs edge which indicate the following
- SD-CARD (blue)
    - Off: No uSD-Card detected
    - On: uSD-Card detected
    - Flashing: Actively logging (writing) to card
- SYSTEM (blue) 
    - Off: System not working at all
    - Flashing: Indicates that FreeRTOS and the tasks are running.
- GNSS (blue) 
    - Off: No GNSS fix
    - Flashing: GNSS fix
- ERROR (red) 
    - Flashing: at least one of the sensors IMU, static pressure, dynamic pressure or GNSS is not working.
    - Sporadic flashing: DGNSS heading fix is briefly missing. 
- ESP (blue)
    - Not used currently

