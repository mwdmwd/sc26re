# SPDX-License-Identifier: AGPL-3.0-or-later

# nRF52833 has intentional POWER/CLOCK and ACL/NVMC register-block overlaps.
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
