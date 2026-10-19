# Home Dice · 3D Dice

A 3D dice roller for the **Waveshare ESP32-S3-Touch-AMOLED-2.16**. Roll with the touchscreen, shake gesture, or the board's GPIO18 button. The Arduino sketch renders the dice locally on the 480×480 AMOLED.

This repository also includes a browser preview that replays pre-rendered frames. It is an interactive visual replay, **not an ESP32 firmware simulator**.

![Dice roll animation](media/dice-roll.gif)

<sub>Animation made from the project's rendered frames. It does not show firmware running on the physical board.</sub>

| Start | Mid-roll | Result |
| --- | --- | --- |
| ![Start face](media/dice-start.png) | ![Dice rotating](media/dice-mid-roll.png) | ![Result six](media/dice-result-6.png) |

[![Multiple dice orientations](media/dice-throw-filmstrip.png)](preview/)

## Hardware

- Waveshare ESP32-S3-Touch-AMOLED-2.16
- CO5300 480×480 AMOLED
- CST9220 touch controller
- QMI8658 IMU
- GPIO18 board button

## Build the firmware

Open `firmware/dice3d/dice3d.ino` as an Arduino sketch. Install an ESP32 Arduino core and the libraries used by the sketch: Arduino_GFX, SensorLib (QMI8658), and the CST92xx touch driver. Select the Waveshare ESP32-S3 target with 16 MB flash and OPI PSRAM. The exact board package options are in [`firmware/dice3d/BUILD.md`](firmware/dice3d/BUILD.md).

To compile without uploading:

```sh
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,FlashSize=16M,USBMode=hwcdc,UploadSpeed=460800" \
  firmware/dice3d
```

No board upload is performed by the preview or host test.

## Run the browser replay

Serve this repository over HTTP, then open `/preview/`:

```sh
python3 -m http.server 8000
```

Choose a target face, roll, or step through frames. The preview assets are recorded frames; they do not emulate the display bus, touch controller, IMU, or firmware execution.

## Native renderer check

```sh
c++ -O2 -std=c++17 -o /tmp/dice-host firmware/dice3d/host_test.cpp
/tmp/dice-host /tmp
```

This checks the renderer's face orientation and writes sample PPM images to `/tmp`.

## Project structure

- `firmware/dice3d/` — Arduino firmware, SDF renderer, native host check
- `preview/` — browser replay UI and rendered frames

## 中文

这是为 **Waveshare ESP32-S3-Touch-AMOLED-2.16** 制作的 3D 骰子。可以通过触摸、摇晃或 GPIO18 按键掷骰。Arduino 固件在 480×480 AMOLED 上本地绘制骰子。

仓库也提供浏览器预览：选择落点、播放或逐帧查看预渲染画面。**它是交互式画面回放，不是 ESP32 固件仿真器。**网页不会运行 Arduino 固件，也不会模拟屏幕总线、触摸芯片或 IMU。

上面的动画和截图均由项目渲染帧生成，不代表真机屏幕录像，也不证明固件已在当前板子上运行。

从仓库根目录运行 `python3 -m http.server 8000`，访问 `/preview/`。固件编译参数及原生渲染器检查命令见上方英文说明和 `firmware/dice3d/BUILD.md`。

## Source and license

ESP32 Gallery-authored firmware, browser code, rendered preview frames, and
documentation in this repository are released under the
[MIT License](LICENSE). Third-party libraries retain their own licenses; see
the build documentation and upstream package metadata before redistribution.

本仓库中由 ESP32 Gallery 创作的固件、网页代码、渲染预览帧与文档采用
[MIT License](LICENSE)。第三方库仍适用各自许可证，分发前请查阅构建文档和上游包元数据。
