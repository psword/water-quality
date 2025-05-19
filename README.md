# water-quality
 
 This code relates to the research project topic _Development of an Open-Source Real-Time Water Quality Monitoring Device: A Modular Approach_.

 Releases are linked here:

 [![DOI](https://zenodo.org/badge/931396380.svg)](https://doi.org/10.5281/zenodo.15364438)
 
 **General Notes:**

 - I also mainly use PlatformIO w/ Visual Studio Code to compile,
 although I use Arduino IDE as a companion tool. These tools are simply convenient.

**Table of Contents:**  
**Main Code (primarily for deployment):**  
- _[Platform-IO-builds](/Platform-IO-builds)_
  - [Calibration-specific code](Platform-IO-builds/Calibration-Script-Water-Quality)
  - [Code to deploy measurement system](Platform-IO-builds/LilyGo-T-SIM7000G-Water-Quality)

**Secondary Code (primarily for testing with second IDE):**  
- _[Arduino-IDE-Testing](/Arduino-IDE-Testing)_  

**Datasets, Analysis, Logs:**  
- _[Serial-Logs](/Serial-Logs)_  
- _[Other-Data](/Other-Data)_  
  - [Parts Costs](Other-Data/Cost)  
  - [Datasets from testing](Other-Data/Datasets)  
  - [Electronics Schematics and Images](Other-Data/Schematics)  
  - [GSM Module Specific Logs](Other-Data/sim-tests)
    
---

**Libraries Needed:**
	
 [paulstoffregen/OneWire@^2.3.8](https://registry.platformio.org/libraries/paulstoffregen/OneWire)
 
 [milesburton/DallasTemperature@^3.11.0](https://registry.platformio.org/libraries/milesburton/DallasTemperature)
	
 [wollewald/ADS1115_WE@^1.5.4](https://registry.platformio.org/libraries/wollewald/ADS1115_WE)
 
 [bblanchon/ArduinoJson@^7.3.1](https://registry.platformio.org/libraries/bblanchon/ArduinoJson)

 [vshymanskyy/TinyGSM@^0.12.0](https://registry.platformio.org/libraries/vshymanskyy/TinyGSM)

 [naguissa/uRTCLib@^6.9.2](https://registry.platformio.org/libraries/naguissa/uRTCLib/installation)

 [knolleary/PubSubClient@^2.8](https://registry.platformio.org/libraries/knolleary/PubSubClient/installation)


### PlatformIO.ini

[env:esp32dev]

platform = espressif32@~5.0.0

board = esp32dev

framework = arduino

- lib_deps = 
	- paulstoffregen/OneWire@^2.3.8
	- milesburton/DallasTemperature@^3.11.0
	- wollewald/ADS1115_WE@^1.5.4
	- vshymanskyy/TinyGSM
	- naguissa/uRTCLib@^6.9.2
	- naguissa/uEEPROMLib@^1.2.1
	- bblanchon/ArduinoJson@^7.3.1
	- knolleary/PubSubClient@^2.8

[platformio]

- description = ESP32 board connected to three sensors
 
