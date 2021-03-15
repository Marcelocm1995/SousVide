# SousVide
This project is a thermo-circulator for cooking meats, fishes, etc. It's consists, basically, in a resistance thats heat up water at pre-determinatade temperature and time(or not).

# Bill Of Materials
* 500 Watts resistance
* DS18B20 Temperature sensor
* Water pump (from aquarium)
* 5V power supply (from smartphone charger)
* BluePill (STM32F103C8T6) 
* Rotary Encoder
* OLED DISPLAY
* 5V Relay
* 1N4148 Diode
* 5V Beep
* 2x BC548 transistors
* 3x 4k7 resistors
* 5mm LED
* 1k resistor


# How It Works:
This project have two operations modes:
* Heat(Aquecimento), will only heat water and keep at user determinatade temperature for indeterminatade time.
* Cooking (Cozinhar), will heat and keep water temperature for user determinatade temperature and time.

# Software:
It uses cubeMX for generate a pre-code. Compiler Keil uVision, but you can easily migrate to another suported compiler by cubeMX, like IAR.
The software first shows on display the operations modes for user and wait for a choice. Case Heat (aquecimento) mode is chose, display will ask for witch temperature user want to set, and will start the heating process. In this mode, when temperature achieve the setpoint some beeps will be emitted.
Case Cooking (Cozinhar) mode is chose, display will ask for witch temperature user want to set, and then witch time user want to set. In this mode, when timer done some beeps will be emitted.
To cancel any of these process is necessary to hold on encoder button for some seconds.

**You need to extract the "Drivers.zip" to compile this software**

## Control
It uses a simple bang-bang control (on/off) with a offset of +/- 0.2°C, but if you want it have a PID mode (at line 48 of main.c, uncalibrated).

## Debug
It have a USB CDC for debug, to enable this uncomment line 58 of main.c

# Hardware:
This hardware uses the schematic bellow:

<a href="https://imgur.com/b3Bf7M9"><img src="https://imgur.com/b3Bf7M9.jpg" title="source: imgur.com" /></a>

* The signal pin is a PWM working with 0% or 100% in bang-bang mode, in PID mode you will need a solid-state relay.
* Use a power supply at least 250mA.
* Pay attention with current capacite of conductors in heater part 
* Use a shielded waterproof DS18B20

# Enclosure
I made a 3D printed enclosure for this project, STL's and STEP's are avaiable for download on this repository.
![alt text for screen readers](/Images/Capturar.PNG "Text to show on mouseover")
![alt text for screen readers](/Images/Capturar2.PNG "Text to show on mouseover")

# Assembly
The middle part of enclosure needs to seal with silicone, like bellow:
![alt text for screen readers](/Images/1615595364210.jpg "Text to show on mouseover")

# Final project
![alt text for screen readers](/Images/IMG_20210220_184832.jpg "Text to show on mouseover")
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/m6A6xz97m30/0.jpg)](https://www.youtube.com/watch?v=m6A6xz97m30)

https://www.youtube.com/watch?v=m6A6xz97m30
		
