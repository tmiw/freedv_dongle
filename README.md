# Introduction

freedv_dongle is a project to accelerate FreeDV processing for certain devices that may not be able to run the FreeDV application or Codec2 libraries normally.
Currently, development is for the [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) board.

# Compiling

Install arm-none-eabi-gcc and arm-none-eabi-binutils, then run the following:

```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```

# Flashing onto the device

1. Open Teensy Loader (which comes with Teensyduino).
2. Choose File->Open HEX File and select freedv_dongle.hex (inside build/src).
3. Plug the Teensy board into an open USB port and push its button to trigger firmware upload and reboot.

# Integration with your own application

TBD. See src/dongle_protocol.h for the messages that are supported by the firmware.
