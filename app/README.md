<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# SC26re firmware application

This directory contains the Zephyr application, out-of-tree Ibex board support, drivers, &c.

The primary target is `steam_controller_ibex/nrf52833`. There is also a `bbc_microbit_v2` target for experimentation. Support for other nRF528xx boards could easily be added in principle.

## Targets

### `steam_controller_ibex/nrf52833`

Out-of-tree board for the controller board.

- nRF52833 QIAA, 512 KiB flash, 128 KiB RAM
- direct GPIO buttons for A, B, Steam, View, and Menu
- SLG4L48185 GreenPAK-backed digital inputs for the rest
- LSM6DSV16X IMU
- Olympus touchpad devices
- six-channel SAADC scan for sticks and triggers
- RGBW LED on PWM1
- Valve bootloader-compatible flash layout

#### Bootloader entry chord

To stay in Valve's resident bootloader instead of loading the application, hold:

```
View + Menu + A
```

then press Steam to power the controller on.

### `bbc_microbit_v2`

Development target using Zephyr's in-tree BBC micro:bit v2 board, plus a small app overlay. It is useful for exercising the ESB/Bluetooth HID, shell (though over direct UART instead of CDC-ACM) etc without involving Ibex hardware.

## Building

The repository pins upstream Zephyr 4.4.1 and Zephyr SDK `1.0.1`.

Install the ARM SDK subset:

```sh
make zephyr-sdk-arm
```

Build the micro:bit target:

```sh
make app-microbit
```

Build the Ibex target:

```sh
make app-ibex
```

The Ibex package output is a `.fw` container with the header expected by the bootloader. The application itself is linked at `0x08000`.

Direct `west` builds are also possible:

```sh
cd zephyr
ZEPHYR_SDK_INSTALL_DIR=../sdk/zephyr-sdk-1.0.1 \
  west build -p auto -b bbc_microbit_v2 ../app \
  -d build-app-microbit -- \
  -DEXTRA_CONF_FILE="ble.conf;esb.conf"

ZEPHYR_SDK_INSTALL_DIR=../sdk/zephyr-sdk-1.0.1 \
  west build -p auto -b steam_controller_ibex/nrf52833 \
  ../app -d build-app-ibex -- -DBOARD_ROOT=../app \
  -DEXTRA_CONF_FILE="ble.conf;esb.conf"
```

## Flashing Ibex

For a controller already in Valve ISP bootloader mode:

```sh
make app-ibex-bl-flash IBEX_BL_PORT=/dev/ttyACM0
```

For an initial flash with bootloader entry triggered over USB HID reports:

```sh
make app-ibex-flash IBEX_SERIAL=<controller serial>
```

The flasher requests ISP mode from the running firmware, waits for the bootloader serial device, matches it by serial, sends the package, and reboots.

Once the custom firmware is running, the Ibex USB runtime exposes a composite device with:

- CDC ACM shell
- normal Valve Triton HID interface (`28de:1302`)

To return to ISP mode from CFW, either use the HID helper path or run:

```text
steamctl power reboot_isp
```

from the shell.

`flash.py update` can preserve the CFW settings partition during normal CFW-to-CFW updates by saving the raw Zephyr NVS area before entering the bootloader and restoring it afterward. Starting from the bootloader directly cannot do that.

## Flash layout

Valve's bootloader at:

```text
0x00000-0x07fff
```

The custom application starts at:

```text
0x08000
```

The CFW settings partition currently lives near the top of the application area:

```text
0x78000-0x7bfff
```

The bootloader package header page is reserved around:

```text
0x7cfe0
```

The stock firmware storage area is left alone:

```text
0x7d000-0x7ffff
```

The CFW tries not to write to the stock firmware storage partition. The current layout is chosen so switching firmware preserves the original firmware's state.

Valve's bootloader erases `0x08000-0x7cfff` at the start of an update, which covers the CFW settings storage at `0x78000-0x7bfff`. So OFW storage should survive CFW/OFW switches, while preserving CFW settings requires support from the host `flash.py` tool.

## Shell

The `steamctl` shell is available over:

- the micro:bit's usual UART, on the micro:bit target
- USB CDC ACM, on Ibex

Useful commands:

```text
steamctl status
steamctl power reboot
steamctl power reboot_isp
steamctl radio ble
steamctl radio esb
steamctl radio esb_bond <proteus_uuid> <ibex_uuid> [serial]
```

`reboot_isp` is Ibex-only. It writes the flag words used by Valve's ISP reboot path and then performs a reboot.

## Radio

The firmware includes BLE, ESB, and USB support. Bluetooth uses Zephyr's Host and Free `BT_LL_SW_SPLIT` controller. ESB uses a direct nRF RADIO backend. The build does not include NCS, MPSL, the SoftDevice Controller, nrfxlib, or any nonfree Nordic libraries.

BLE and ESB share the radio, so only one radio personality runs per boot. Use the `steamctl radio` commands or the boot chords to select one, changing the radio personality reboots the controller. If coexistence is needed in the future, it will be added without MPSL, how hard could it be?

## Power

The current firmware has a power-button long-hold system-off path and sleep mode.
Battery level, charger detection, charging behavior, and OFW-matching idle current still need verification.

## Haptics

The micro:bit target has a tiny PWM piezo beep for button-press feedback. The Ibex target has its primary touchpad PWM and secondary GRI-v3 I2S actuator paths enabled.
