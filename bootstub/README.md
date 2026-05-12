# Ibex micro:bit boot stub

This is a minimal Cortex-M loader stub for trying the Ibex payload on a BBC micro:bit v2/nRF52833.

The generated image combines:

* `bootstub/stub.c` at `0x00000000`
* `IBEX_FW_69FE17FF.fw.payload.bin` at `0x00008000`

Build (from root directory):

```sh
make bootstub
```

Output is:

```text
bootstub/build/ibex-microbit.elf
bootstub/build/ibex-microbit.hex
```
