# Steam Controller NCS manifest

This directory is an attempt to reconstruct an NCS/Zephyr checkout close to the Steam Controller firmware.

For west, `manifest/` is the manifest repository. The actual SDK projects are managed by west as sibling paths inside this directory, including `nrf/` and the actual Zephyr source checkout at `zephyr/`.

Setup:

```sh
cd zephyr
west init -l manifest
west update
west manifest --validate
west list nrf zephyr -f '{name:12} {path:30} {revision}'
```

Notes:

- `sdk-nrf` is pinned to public NCS v2.9.0 commit `7787b264984022cda64d9629278942053e6462a5`
- Keeping the manifest in `manifest/` avoids Zephyr module discovery treating the workspace root itself as a module
