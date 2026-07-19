# Valve NVS compatibility fixture

`valve_nvs_3_7_99_wrap.bin` is a synthetic 12 KiB NVS image produced by the unmodified NVS implementation from Nordic's `sdk-zephyr` commit `1f8f3dc291420c70cd39e77a5cdc954561d4a08f` (`v3.7.99-ncs2`). It contains no controller dump data.

The native simulator used three 4096-byte sectors, four-byte write alignment, erase value `0xff`, and no data CRC. The writer stored six fabricated calibration Settings records using the legacy name-count/value/name ordering, advanced to the next sector, replaced the left-trigger value, wrote and deleted an unrelated record, and advanced twice more. The resulting image therefore exercises a complete sector wrap and garbage collection using the real legacy implementation.
