# ARM DevSummit 2021 - Edge Impulse Portenta workshop

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Arduino Portena H7 development board. This device supports all Edge Impulse device features, including ingestion, remote management and inferencing.

## Requirements

### Hardware

* [Arduino Portenta H7](https://store.arduino.cc/portenta-h7) development board.
* [Arduino Portenta Vision Shield](https://store.arduino.cc/portenta-vision-shield).

### Tools

macOS and Linux users can use either the Arduino CLI and IDE to compile the firmware.
Windows users will need the Arduino IDE.

#### Arduino CLI (macOS/Linux only)

Use following link for download and installation procedure:

* [Arduino CLI](https://arduino.github.io/arduino-cli/installation/).

The Edge Impulse firmware depends on some libraries and the Mbed core for Arduino. These will be automatically installed if you don't have them yet.

#### Arduino IDE (Windows/macOS/Linux)

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software) for your Operating System.
1. In Tools -> Board -> Boards Manager, search for `portenta` and install the **Arduino Mbed OS Portenta Boards v2.5.2**.
1. In Arduino Menu -> Preferences, check the location of the **preferences.txt** file (ie: /Users/aureleq/Library/Arduino15/).
1.  Copy the `boards.local.txt` and `platform.local.txt` files into the Arduino Mbed Portenta directory, for instance:
`/Users/aureleq/Library/Arduino15/packages/arduino/hardware/mbed_portenta/2.5.2`.

## Building the application

### With Arduino CLI

1. Build the application:

    ```
    ./arduino-build.sh --build
    ```

1. Flash the application:

    ```
    ./arduino-build.sh --flash
    ```

### With Arduino IDE

1. Open the `workshop-devsummit2021-portenta.ino`, select the **Arduino Portenta H7 (M7 core)** board and the Flash Split **1 MB M7 +  1 MB M4**.
1. Build and flash the application using the **Upload** button.

## Using your own exported `C/C++` library from

Extract the contents of the exported `C/C++` library and replace `edge-impulse-sdk`, `tflite-model` and `model-parameters` in the project
accordingly. Next rebuild the application.

## Known Issues

- Image corruption after audio capture when frame buffer is placed in SDRAM (`EI_CAMERA_FRAME_BUFFER_SDRAM`).

A simple work-around is to avoid image capture after audio capture, otherwise reset the board.

## Troubleshooting

* Not flashing? You can double tap the button on the board to put it in bootloader mode.
* Invalid DFU suffix signature?

    ```
    dfu-util: Invalid DFU suffix signature
    dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
    dfu-util: Cannot open DFU device 2341:035b
    dfu-util: No DFU capable USB device available
    Upload error: Error: 2 UNKNOWN: uploading error: uploading error: exit status 74
    ```

    Having the above issues? Then copy `20-arduino.rules` to `/etc/udev/rules.d/` and try again.