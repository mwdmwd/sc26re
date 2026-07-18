# Steam Controller Zephyr manifest

This directory contains a minimal upstream Zephyr workspace used to build the custom firmware.

For west, `manifest/` is the manifest repository. The Zephyr source and the six required modules are managed as sibling paths inside this directory.

Setup:

```sh
cd zephyr
west init -l manifest
west update
west manifest --validate
west list -f '{name:16} {path:36} {revision}'
```

The manifest pins:

- upstream Zephyr 4.4.1,
- CMSIS 5 and CMSIS 6,
- Nordic and ST hardware abstraction layers,
- Mbed TLS and TF-PSA-Crypto.

There are no NCS, nrfxlib, MPSL, SoftDevice Controller, or any other nonfree Nordic projects.
Keeping the manifest in `manifest/` avoids Zephyr module discovery treating the workspace root itself as a module.
