# BME 463 — Raspberry Pi Pico labs and final project

Labs and a final project for BME 463 (biomedical signal processing on a microcontroller),
written in C/C++ with the Raspberry Pi Pico SDK.

## Final project: pacemaker pulse detection in ECG signals

`BME463_Receiver/Labs/Final_Project.c` detects pacemaker spikes in ECG recordings with a
modified Pan–Tompkins pipeline tuned for the very fast spike:

```
high-pass filter → derivative → squaring → moving-window integration → adaptive threshold
```

It was tested on MIT-BIH Arrhythmia Database records 102, 104, 107, 113 and 217. A GPIO pin
goes high when a pacemaker pulse is detected, and a second pin gives an oscilloscope timing
flag for measuring processing latency.

## Repository layout

| Path | What it is |
|---|---|
| `BME463_Receiver/` | Main Pico project: lab framework, drivers and all labs |
| `BME463_Receiver/Labs/` | `lab03` – `lab12` and `Final_Project` |
| `BME463_Receiver/Drivers/` | MCP4728 DAC (I2C), seven-segment display and filter drivers |
| `BME463_Receiver/mini_rpnet_weights.h` | Auto-generated neural-network weights |
| `BlinkLED`, `BlinkLEDNEW`, `HelloWorld` | First Pico bring-up programs |
| `SineWave`, `SineWave_LEDBlink` | Sine-wave generation over SPI, with and without an LED blink |

Choose which lab to build by enabling one `#define` in `BME463_Receiver/Active_Lab.h`.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) and CMake.

```
cd BME463_Receiver
mkdir build && cd build
cmake ..
make
```

## Credits

The lab framework (`main.c`, `main.h`, the drivers and the lab templates) was provided by the
course staff and is credited in the file headers. The final project is my own work.
