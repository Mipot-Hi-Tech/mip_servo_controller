<!-- For .md file development refers to https://docs.github.com/en -->
# [MIP](https://mipot.com/en/products/?cat=110) Series Design and Firmware Example for Dual Core

## Wireless Triple Stepper/Servo Controller 

Designed to remote control up to three axis mechatronic system.
Build with [MIPOT](https://www.mipot.com) [32001506DEU](https://mipot.com/en/products/mip-series/dual-core/32001506deu/).

## Compatibility

 - [32001506AEU](https://mipot.com/en/products/mip-series/dual-core/32001506aeu/) - 868 MHz Wireless M-Bus
 - [32001506BEU](https://mipot.com/en/products/mip-series/dual-core/32001506beu/) - 868 MHz LoRaWAN
 - [32001506CEU](https://mipot.com/en/products/mip-series/dual-core/32001506ceu/) - 868 MHz LoRa Mipot
 - [32001506DEU](https://mipot.com/en/products/mip-series/dual-core/32001506deu/) - 868 MHz LoRa Modem
 - [32001506BUS](https://mipot.com/en/products/mip-series/dual-core/32001506bus/) - 915 MHz LoRaWAN

> [!TIP]
> MIP C library is avaliable in the [Official Repository](https://github.com/Mipot-Hi-Tech/mip).

## Board Overview

![img_cover](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img001.png)
![img00](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img012.png)
![img01](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img002.png)

## Top Side of the board

![img1](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img003.png)

## Bottom Side of The board

![img2](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img004.png)

## Board with 4 layers for EMI Compliance

![img3](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img005.png)

## Build with Kicad

![img4](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img006.png)

## Mode of Operation

![img5](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img007.png)

## Closeup

![img6](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img008.png)

![img7](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img009.png)

## SPI commonucation between microcontroller and TMC2130 Drivers

![img8](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img010.png)

## Embedded shell for Hardware Debug

![shell](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/shell.gif)

## Folder content and description

- **firmware:** Source code of the microcontroller written in C.
	- **Core:** The application files, system and peripheral configuration.
	- **freertos:** The Freertos kernel. *Keep read only*.
	- **freertos_cli:** The Freertos command line interface source code. *Keep read only*.
	- **wl-lib:** STM32 HAL and LL Drivers. *Keep read only*.
	- **trinamic_lib:** Driver of TMC2130 stepper controller.
	- **miplib** Driver of MIP series modules.
- **img:** Development and test images of the project.
- **kicad:** hardware design files.Source files of schematic and PCB.
	- **lib:** Symbols and footprints of the components.
- **production:** Production files. Gerbers, BOM and Stencil.
- **doc:** Schematic of the board. PDF format.

## Firmware Overview

Based on [Freertos](https://www.freertos.org).
![img9](https://github.com/Mipot-Hi-Tech/mip_servo_controller/blob/master/img/img011.png)

## Software needed for development

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) **1.17.0**
- [KICAD](https://www.kicad.org/) **8.0.5**
- [GIT](https://git-scm.com/) **2.46.1.windows.1**
- [Tera Term](https://teratermproject.github.io/index-en.html) **5.3 x86**

## Tools needed for development

- Programmer [STLINK](https://www.st.com/en/development-tools/st-link-v2.html)
- Serial Cable [TTL-232R-3V3](https://ftdichip.com/products/ttl-232r-3v3/)

## License

Shown in the LICENSE.md file

## Important Information

> [!CAUTION]
> Manual soldering of SMD components requires high level of manual skills.

> [!CAUTION]
> This project is a hardware/firmware solution example on MIP Series.
> The main goal is to give a starting point on MIP Series Development.
> There are better open source solutions to control steppers motors.

> [!CAUTION]
> This project is provided "AS IS" with no warranties. 