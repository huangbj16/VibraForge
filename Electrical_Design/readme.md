# Electrical Engineering Section ##

The electrical engineering section includes PCB design files for control units and vibration units, and main programs for microcontrollers. All PCB design were initially finished in [EasyEDA](https://easyeda.com/) and later exported to different formats. If you want to manufacture the board with PCB factories such as [JLCPCB](https://jlcpcb.com/) or [PCBWay](https://www.pcbway.com/), simply use the Gerber files. If you want to revise the design, Import the design files into EasyEDA Editor or Altium Designer.

Control units include the large unit and the small unit, with different ESP32 microcontrollers. The large unit uses [Adafruit ESP32 Feather V2](https://learn.adafruit.com/adafruit-esp32-feather-v2). The small unit uses [Adafruit QT Py ESP32-S3](https://learn.adafruit.com/adafruit-qt-py-esp32-s3). The Arduino programs are very similar, except the software serial pins are configured differently.

Arduino Library Rrquirements:
- [Arduino ESP32 support](https://github.com/espressif/arduino-esp32)
- [SoftwareSerial for ESP32](https://github.com/plerup/espsoftwareserial)

Vibration units include LRA and VCA, all using the PIC16F18313 8-pin microcontroller. The main program is the same across all vibration units. To program / reprogram the MCU, you need to download the [MPLAB X IDE](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide) and buy the  [MPLAB® PICkit™ 5 In-Circuit Debugger](https://www.microchip.com/en-us/development-tool/pg164150). add the main program (`Vibration_Unit.c`) and the NeoPixel driver (`neopixel_control.c`, `neopixel_control.h`) to an MPLAB X project, connect the programming pins from the debugger to the top of the PCB board, following this [quickstart guideline](https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/Brochures/MPLAB-PICkit-5-In-Circuit-Debugger-Quick-Start-Guide-50003478.pdf).

- Control_Unit
  - Large_Unit
  - Small_Unit
  - Main_Program_Arduino
    - BLE_peripheral_ESP32V2
    - BLE_peripheral_QTPyS3
- Vibration_Unit
  - Board_LRA
  - Board_VCA
  - Main_Program_MPLABXIDE
    - tests
  - Schematic



