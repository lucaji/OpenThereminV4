# OpenTheremin v4.x platformio IDE

a new fork for the open source Theremin instrument software and hardware project by [GaudiLabs](https://github.com/GaudiLabs/OpenThereminV4). Adapted for [platformio](https://platformio.org/) IDE. Schematics, printed circuit board (PCB) design, bill of materials (BOM) and Arduino UNO compatible software for the OpenTheremin v4.x

> This repository brings an refactored, cleaned, fully commented and slightly optimized firmware for all OpenTheremin v4.x boards.

Lots of experimentation with this firmware is possible, check `build.h` for options.


## Waveform interpolation

Three waveform interpolation algorithms are included for experimenting with the 328p DDS generation:

- NONE (LEGACY)
- LINEAR
- HERMITE
- CATMULL-ROM (pushing the limits of the 328p - computational hungry!)

There is an interactive mode where each mode can be selected by muting/unmuting the Theremin. Using the NONE (LEGACY) mode, gives anyway a clean spectrum content on the audio output, fast responsiveness and an extended pitch range because of the optimizations that were already in place in the original OpenTheremin firmware.

## Serial port configuration

- Off (saves resources!)
- DEBUG (gives some state information)
- MIDI (emits MIDI data at 31250bps)

## Open Source Theremin based on the Arduino Platform

OpenTheremin is an arduino based real Theremin instrument. The legendary music instrument was invented by [Leon Theremin](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://it.wikipedia.org/wiki/Lev_Sergeevi%25C4%258D_Termen) back in 1920.

The Theremin is played with both hands around two antennas, one to control the pitch and the other for volume expressivity.

The fully assembled version 4.x of OpenTheremin leverages two heterodyne VCO oscillators (voltage controlled) to measure the distance of the hands to each antenna while playing the instrument, by comparing those frequencies with two fixed frequency local oscillators.

The resulting signals (beating) are fed to and processed by a AtMega328P microcontroller which applies linearization and filtering in real-time, generating the sound reflecting the player's hands movements in the thin air.

There is an fast audio DAC onboard which delivers high quality digital audio, generated via a DDS (Direct Digital Synthesis) synth accessing serveral user-presettable and customizable wavetables to achieve different timbres.

## Instructions

This repo favours [VSCode + Platformio IDE](https://platformio.org/install/ide?install=vscode) as development environment instead of the Arduino IDE, still retaining compatibility though.

To fully enjoy a better development experience, it is advised to install Visual Studio Code and add the platformio IDE plug-in. After the latter has been installed, just opening the folder in VS Code will activate the platformio IDE.

## Ordering preassembled and tested boards

For more info on the open source project and on availability of ready made shield see:

https://www.gaudi.ch/OpenTheremin/

## Limits

> please be aware that the included DDS sound generator algorithm on a 16MHz 328p (Arduino UNO compatible) which is hardware driven by a two etherodyne RF VCO (voltage controlled oscillators) is already pushing the limits of the internal timers and interrupt pressure.

The lesser additional options you include in the build, the better sound quality:

- keep the serial port disabled if you don't need it.
- disable the CV/GATE outputs if you don't need them.
- select the desired interpolation mode if you like one - OFF otherwise!

#### LAST REVISION:

July 2025 by Luca Cipressi (lucaji.github.io)
