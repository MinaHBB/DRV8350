# DRV8350 C Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

This is a C library for the DRV8350, a gate driver designed for BLDC motors. The library is specifically tailored to work with SPC5 MCUs, but it can be adapted to work with other MCUs by modifying the "HAL functions."

## Features

- Provides enums for different bit fields of registers.
- Includes a struct containing all the registers of the DRV8350 IC.
- Allows updating the values of the struct containing all the registers using `DRV8350_updateObject()`.
- Offers functions for reading and writing values to the IC using `DRV8350_read()` and `DRV8350_write()`.
- Includes specific functions to modify individual parts of the IC, such as `DRV8350_setPwmMode()` to change the PWM mode of DRV8350.

## Usage

1. Ensure proper setup of HAL functions according to the target MCU.
2. Include the library in your project.
3. Utilize the provided functions to interact with the DRV8350 IC.

## Documentation

For more detailed information, refer to the comments within the library code.

## Contribution

Contributions and improvements to the library are welcome. Feel free to submit pull requests or open issues for any suggestions or bug reports.
