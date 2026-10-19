# Firmware build and validation

## Board target

Waveshare ESP32-S3-Touch-AMOLED-2.16, CO5300 480×480 display, CST9220 touch, and QMI8658 IMU.

Compile (without uploading):

```sh
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,FlashSize=16M,USBMode=hwcdc,UploadSpeed=460800" \
  firmware/dice3d
```

The sketch requires Arduino_GFX, SensorLib's QMI8658 driver, and a CST92xx touch driver compatible with `TouchDrvCSTXXX.hpp`. Library versions can affect the board APIs; use the Waveshare board examples as the compatibility reference.

## Renderer options

At the top of `dice3d.ino`:

```c
#define DICE_USE_SDF 1   // SDF ray marching; higher image quality
#define SDF_RES      240 // internal rendering resolution
```

These can be overridden with compiler definitions.

## Native host check

No ESP32 board is required:

```sh
c++ -O2 -std=c++17 -o /tmp/dice-host firmware/dice3d/host_test.cpp
/tmp/dice-host /tmp
```

The program checks the upward-facing result for each dice value, reports renderer timings, and writes six PPM images to `/tmp`.

## Hardware notes

- The firmware enables a non-blocking serial logging macro. A full USB CDC buffer can stall a sketch if logging is changed to use blocking writes.
- Keep the display/touch/IMU initialization order in the sketch when adapting the board setup.
- Upload speed is set to 460800 in the FQBN to keep USB CDC uploads reliable on this board.
- The in-browser preview replays rendered frames. It does not model QSPI, touch hardware, sound, or firmware timing.
