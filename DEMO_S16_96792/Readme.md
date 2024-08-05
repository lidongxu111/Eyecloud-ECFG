##Introduction
 - ECFG_ SDK is a software development toolkit developed by EyeCloud, which can be used to run devices such as S16-9296, S1-9296, S1-9702, S1-96792, etc.

## Environmental Description
 - Operate System : Windows10 / Windows Server2016 / Ubuntu18.04.
 - Development version : Visual Studio 2015 Release  windows kit 8.1/ CMake 3.10 & Glibc 3.4.26 .

## Directory Structure
 -   |-- DEMO_S16_96792
 -       |-- Readme.md 	// Version description of demo
 -       |-- lib/		// Static library file directory for ecfg_sdk
 -       |-- inc/		// All functions and their introductions in ecfg_sdk
 -       |-- DEMO.sln		// Compile demo through 'vs2015 release' mode under Windows
 -       |-- CMakeLists.txt	// Compile demo through 'cmake' under ubuntu
 -       |-- libusb/		// The library that USB drivers rely on
 -       |-- opencv/		// The library that image display processing relies on
 -       |-- pthread/		// The libraries that multithreading relies on
 -       |-- DEMO/
 -          |-- config/		// Directory of configuration files for UGrab Studio devices and modules
 -          |-- ECFG.mvcmd      // Running firmware for 9296 and 96792 series devices
 -          |-- moviUsbBoot.exe	// Applications required for burning firmware under windows
 -          |-- moviUsbBoot            // Applications required for burning firmware under ubuntu
 -          |-- ecfg.exe        // Executable programs generated under Windows
 -          |-- ecfg_demo        // Executable programs generated under Ubuntu

## Windows USB Driver install
 - * Before running ecfg_sdk demo program.
 - * Only under windows need to install usb driver.
 - Double click 'USB_Driver/USB_Driver_Installer.bat'.

## Ubuntu install
 - $ sudo apt-get install cmake libusb-1.0-0-dev libopencv-dev

## Usage
 - 1.Firmware:
 Firmware to be downloaded should be in the executable program file.

```
 SDK will automatically Use the tool 'moviUsbBoot.exe' or 'moviUsbBoot' to download the firmware 'ECFG.mvcmd'.
```

## Compile Demo
 - *Winodws:
 - Remove comment out file 'inc/includes.h' line11 '#define _WINDOWS'.
 - Double click on the DEMO.sln file, open the ecfg_sdk project, select the 'release' mode, click on 'Generate' - 'Generate Solution', and generate the static library file of ecfg_sdk.
 - *Ubuntu:
 - Comment out file 'inc/includes.h' line11 '// #define _WINDOWS'.
 - Input command line:
 - $ cd build/
 - $ cmake ..
 - $ make
 - 
## Run:

```
 Make sure the usb driver is installed.
 Get computer, device and cameras connected correctly.
 windows:
 Double click 'ecfg.exe'in "Demo\".
 ubuntu:
 input:
 $ sudo ./ecfg_demo
