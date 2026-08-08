# SPDX-License-Identifier: AGPL-3.0-or-later
SC26RE_MAIN_INCLUDED := 1

# Zephyr SDK / build options
WEST ?= west
ZEPHYR_WORKSPACE ?= $(CURDIR)/zephyr
ZEPHYR_SDK_VERSION ?= 1.0.1
ZEPHYR_SDK_DIR ?= $(CURDIR)/sdk/zephyr-sdk-$(ZEPHYR_SDK_VERSION)
ZEPHYR_SDK_DOWNLOAD_DIR ?= $(CURDIR)/sdk/downloads
ZEPHYR_SDK_RELEASE_URL ?= https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v$(ZEPHYR_SDK_VERSION)
ZEPHYR_SDK_MINIMAL_ARCHIVE := zephyr-sdk-$(ZEPHYR_SDK_VERSION)_linux-x86_64_minimal.tar.xz
ZEPHYR_SDK_ARM_ARCHIVE := toolchain_gnu_linux-x86_64_arm-zephyr-eabi.tar.xz
ZEPHYR_SDK_STAMP := $(ZEPHYR_SDK_DIR)/.arm-toolchain-installed
ZEPHYR_SDK_CMAKE_CONFIG := $(ZEPHYR_SDK_DIR)/cmake/Zephyr-sdkConfig.cmake
ZEPHYR_SDK_GNU_DIR := $(ZEPHYR_SDK_DIR)/gnu
ZEPHYR_SDK_ARM_GCC := $(ZEPHYR_SDK_GNU_DIR)/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc
ZEPHYR_TOOLCHAIN_VARIANT ?= zephyr
ZEPHYR_WEST_CONFIG := $(ZEPHYR_WORKSPACE)/.west/config
ZEPHYR_WEST_UPDATE_STAMP := $(ZEPHYR_WORKSPACE)/.west/sc26re-update.stamp
ZEPHYR_PATCH_STATE := $(ZEPHYR_WORKSPACE)/.west/sc26re-patches.applied
ZEPHYR_MANIFEST := $(ZEPHYR_WORKSPACE)/manifest/west.yml
ZEPHYR_PATCH_BASE := manifest-rev
ZEPHYR_PATCH_DIR := $(CURDIR)/patches
ZEPHYR_PATCHES := $(sort $(wildcard $(ZEPHYR_PATCH_DIR)/*.patch))
# App build options
PRISTINE ?= 0
ZEPHYR_PRISTINE := $(if $(filter 1 yes true always,$(PRISTINE)),always,auto)
BLE ?= 1
CDC_DIAG ?= 0
.PRECIOUS: $(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_MINIMAL_ARCHIVE) $(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_ARM_ARCHIVE)
# Don't pollute ~ with Zephyr's cache
ZEPHYR_USER_CACHE_DIR ?= $(ZEPHYR_WORKSPACE)/zephyr-cache
ZEPHYR_BUILD_ENV := \
	CCACHE_DISABLE=1 \
	XDG_CACHE_HOME=$(ZEPHYR_USER_CACHE_DIR) \
	ZEPHYR_TOOLCHAIN_VARIANT=$(ZEPHYR_TOOLCHAIN_VARIANT) \
	ZEPHYR_SDK_INSTALL_DIR=$(ZEPHYR_SDK_DIR)
# Boot stub
BOOTSTUB_BUILD_DIR ?= $(CURDIR)/bootstub/build
BOOTSTUB_APP_PAYLOAD ?= $(CURDIR)/IBEX_FW_69FE17FF.fw.payload.bin
BOOTSTUB_PATCHED_PAYLOAD ?= $(BOOTSTUB_BUILD_DIR)/IBEX_FW_69FE17FF.microbit.payload.bin
BOOTSTUB_APP_BASE ?= 0x8000
BOOTSTUB_CC ?= arm-none-eabi-gcc
BOOTSTUB_OBJCOPY ?= arm-none-eabi-objcopy
BOOTSTUB_CFLAGS := \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-Os \
	-ffreestanding \
	-fno-builtin \
	-fno-common \
	-ffunction-sections \
	-fdata-sections \
	-Wall \
	-Wextra \
	-DAPP_BASE=$(BOOTSTUB_APP_BASE)
# --
CLANG_FORMAT ?= clang-format
HOSTCC ?= cc
APP_SPDX_LICENSE ?= SPDX-License-Identifier: AGPL-3.0-or-later
# Flashing and debugging for micro:bit v2
OPENOCD ?= openocd
OPENOCD_INTERFACE ?= interface/cmsis-dap.cfg
OPENOCD_TARGET ?= target/nrf52.cfg
# For Ibex dumping I use a Tigard
STOCK_OPENOCD_INTERFACE ?= openocd/tigard-swd.cfg
STOCK_DUMP_ROOT ?= $(CURDIR)/dumps
STOCK_DUMP_OUT ?= $(STOCK_DUMP_ROOT)/$(shell date -u +%Y%m%dT%H%M%SZ)
BOOTSTUB_RTT_RAM_BASE ?= 0x20000000
BOOTSTUB_RTT_RAM_SIZE ?= 0x20000
BOOTSTUB_RTT_PORT ?= 19021
IBEX_UICR_MAGIC ?= 0xac32a429
IBEX_UICR_BOARD_ID ?= 0x45
ZEPHYR_APP ?= $(CURDIR)/app
ZEPHYR_APP_BUILD_PREFIX ?= build-app
APP_FORMAT_SOURCES := $(shell find "$(ZEPHYR_APP)" -type f \
	\( -name '*.c' -o -name '*.h' -o -name '*.cc' -o -name '*.cpp' -o -name '*.hh' -o -name '*.hpp' \) \
	! -path "$(ZEPHYR_APP)/src/sdl/*" | sort)
IBEX_APP_BUILD_DIR := $(ZEPHYR_WORKSPACE)/$(ZEPHYR_APP_BUILD_PREFIX)-ibex/zephyr
MICROBIT_APP_HEX := $(ZEPHYR_WORKSPACE)/$(ZEPHYR_APP_BUILD_PREFIX)-microbit/zephyr/zephyr.hex
IBEX_BUILD_TIMESTAMP ?= $(shell sh scripts/sc26re-build-timestamp.sh)
SC26RE_BUILD_CONF := $(ZEPHYR_WORKSPACE)/sc26re-build.conf
IBEX_CUSTOM_FW := $(IBEX_APP_BUILD_DIR)/IBEX_CUSTOM_$(IBEX_BUILD_TIMESTAMP).fw
MICROBIT_CONF_FILES := ble.conf;esb.conf;$(SC26RE_BUILD_CONF)
IBEX_CONF_FILES := $(if $(filter 1 yes true,$(BLE)),ble.conf;esb.conf,esb.conf)
IBEX_CONF_FILES := $(if $(filter 1 yes true,$(CDC_DIAG)),$(IBEX_CONF_FILES);ibex-cdc-diag.conf,$(IBEX_CONF_FILES))
IBEX_CONF_FILES := $(IBEX_CONF_FILES);$(SC26RE_BUILD_CONF)
IBEX_BL_PORT ?=
IBEX_SERIAL ?=

ifeq ($(.DEFAULT_GOAL),)
.DEFAULT_GOAL := all
endif

include Makefile.steam-fetch

.PHONY: all
all: help

.PHONY: format
format: app-license-check app-format

.PHONY: format-check
format-check: app-license-check app-format-check

.PHONY: app-license-check
app-license-check:
	@missing=$$(grep -L "$(APP_SPDX_LICENSE)" $(APP_FORMAT_SOURCES) || true); \
	if [ -n "$$missing" ]; then \
		printf 'missing %s:\n%s\n' "$(APP_SPDX_LICENSE)" "$$missing" >&2; \
		exit 1; \
	fi

.PHONY: app-format
app-format:
	$(CLANG_FORMAT) -i $(APP_FORMAT_SOURCES)

.PHONY: app-format-check
app-format-check:
	$(CLANG_FORMAT) --dry-run -Werror $(APP_FORMAT_SOURCES)

.PHONY: test
test: test-valve-nvs test-haptics-stereo

.PHONY: test-valve-nvs
test-valve-nvs:
	$(HOSTCC) -std=c11 -O2 -Wall -Wextra \
		-I"$(ZEPHYR_APP)/src" \
		"$(ZEPHYR_APP)/src/valve_nvs.c" "$(ZEPHYR_APP)/tests/valve_nvs.c" \
		-o "$@"
	"./$@" --legacy-fixture "$(ZEPHYR_APP)/tests/fixtures/valve_nvs_3_7_99_wrap.bin"; ex=$$?; rm "$@"; exit "$$ex"

.PHONY: test-haptics-stereo
test-haptics-stereo: zephyr-workspace
	cd "$(ZEPHYR_WORKSPACE)" && $(ZEPHYR_BUILD_ENV) $(WEST) build -p "$(ZEPHYR_PRISTINE)" \
		-b native_sim "$(ZEPHYR_APP)/tests/haptics_stereo" -d build-test-haptics-stereo
	"$(ZEPHYR_WORKSPACE)/build-test-haptics-stereo/zephyr/zephyr.exe"

.PHONY: bootstub
bootstub: $(BOOTSTUB_BUILD_DIR)/ibex-microbit.hex $(BOOTSTUB_BUILD_DIR)/ibex-microbit.bin

.PHONY: bootstub-flash
bootstub-flash: $(BOOTSTUB_BUILD_DIR)/ibex-microbit.hex
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'init; cortex_m vector_catch reset; reset halt; program $< verify reset exit'

.PHONY: bootstub-flash-halt
bootstub-flash-halt: $(BOOTSTUB_BUILD_DIR)/ibex-microbit.hex
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'init; cortex_m vector_catch reset; reset halt; program $< verify; reset halt'

.PHONY: bootstub-debug-server
bootstub-debug-server:
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'init; cortex_m vector_catch reset; reset halt'

.PHONY: bootstub-run
bootstub-run:
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'init; reset run; shutdown'

.PHONY: bootstub-rtt-server
bootstub-rtt-server:
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'init; reset run; rtt setup $(BOOTSTUB_RTT_RAM_BASE) $(BOOTSTUB_RTT_RAM_SIZE) "SEGGER RTT"; rtt start; rtt channels; rtt server start $(BOOTSTUB_RTT_PORT) 0'

.PHONY: microbit-uicr-read
microbit-uicr-read:
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-f openocd/microbit-uicr-read.cfg

.PHONY: microbit-uicr-provision
microbit-uicr-provision:
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'set IBEX_UICR_MAGIC $(IBEX_UICR_MAGIC)' \
		-c 'set IBEX_UICR_BOARD_ID $(IBEX_UICR_BOARD_ID)' \
		-f openocd/microbit-uicr-provision.cfg

.PHONY: microbit-uicr-erase
microbit-uicr-erase:
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-f openocd/microbit-uicr-erase.cfg

define run_stock_dump
	mkdir -p "$(STOCK_DUMP_OUT)"
	$(OPENOCD) -f "$(STOCK_OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-l "$(STOCK_DUMP_OUT)/openocd.log" \
		-c 'set STOCK_DUMP_DIR {$(STOCK_DUMP_OUT)}' \
		-c 'set STOCK_DUMP_FLASH $(1)' \
		-c 'set STOCK_DUMP_FICR $(2)' \
		-c 'set STOCK_DUMP_UICR $(3)' \
		-f openocd/stock-dump.cfg
	@printf 'stock dump: %s\n' "$(STOCK_DUMP_OUT)"
endef

.PHONY: stock-dump
stock-dump:
	$(call run_stock_dump,1,1,1)

.PHONY: stock-dump-flash
stock-dump-flash:
	$(call run_stock_dump,1,0,0)

.PHONY: stock-dump-ficr
stock-dump-ficr:
	$(call run_stock_dump,0,1,0)

.PHONY: stock-dump-uicr
stock-dump-uicr:
	$(call run_stock_dump,0,0,1)

.PHONY: stock-runtime-dump
stock-runtime-dump:
	mkdir -p "$(STOCK_DUMP_OUT)"
	$(OPENOCD) -f "$(STOCK_OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-l "$(STOCK_DUMP_OUT)/openocd.log" \
		-c 'set STOCK_DUMP_DIR {$(STOCK_DUMP_OUT)}' \
		-f openocd/runtime-dump.cfg
	@printf 'runtime dump: %s\n' "$(STOCK_DUMP_OUT)"

$(BOOTSTUB_BUILD_DIR):
	mkdir -p "$@"

$(BOOTSTUB_BUILD_DIR)/stub.o: bootstub/stub.c | $(BOOTSTUB_BUILD_DIR)
	$(BOOTSTUB_CC) $(BOOTSTUB_CFLAGS) -c "$<" -o "$@"

$(BOOTSTUB_PATCHED_PAYLOAD): $(BOOTSTUB_APP_PAYLOAD) bootstub/patch-ibex-microbit.py | $(BOOTSTUB_BUILD_DIR)
	bootstub/patch-ibex-microbit.py "$<" "$@"

$(BOOTSTUB_BUILD_DIR)/ibex-payload.o: $(BOOTSTUB_PATCHED_PAYLOAD) | $(BOOTSTUB_BUILD_DIR)
	$(BOOTSTUB_OBJCOPY) -I binary -O elf32-littlearm -B arm \
		--rename-section .data=.ibex_app,alloc,load,readonly,data,contents \
		"$<" "$@"

$(BOOTSTUB_BUILD_DIR)/ibex-microbit.elf: $(BOOTSTUB_BUILD_DIR)/stub.o $(BOOTSTUB_BUILD_DIR)/ibex-payload.o bootstub/linker.ld
	$(BOOTSTUB_CC) $(BOOTSTUB_CFLAGS) -nostdlib -T bootstub/linker.ld \
		-Wl,--gc-sections \
		-Wl,--defsym,APP_BASE=$(BOOTSTUB_APP_BASE) \
		-Wl,-Map="$(BOOTSTUB_BUILD_DIR)/ibex-microbit.map" \
		"$(BOOTSTUB_BUILD_DIR)/stub.o" "$(BOOTSTUB_BUILD_DIR)/ibex-payload.o" \
		-o "$@"

$(BOOTSTUB_BUILD_DIR)/ibex-microbit.hex: $(BOOTSTUB_BUILD_DIR)/ibex-microbit.elf
	$(BOOTSTUB_OBJCOPY) -O ihex "$<" "$@"

$(BOOTSTUB_BUILD_DIR)/ibex-microbit.bin: $(BOOTSTUB_BUILD_DIR)/ibex-microbit.elf
	$(BOOTSTUB_OBJCOPY) -O binary "$<" "$@"

.PHONY: zephyr-workspace
zephyr-workspace: $(ZEPHYR_PATCH_STATE)

.PHONY: zephyr-patch-state-force
zephyr-patch-state-force:

$(ZEPHYR_WEST_CONFIG):
	cd "$(ZEPHYR_WORKSPACE)" && $(WEST) init -l manifest

$(ZEPHYR_WEST_UPDATE_STAMP): $(ZEPHYR_MANIFEST) | $(ZEPHYR_WEST_CONFIG)
	cd "$(ZEPHYR_WORKSPACE)" && $(WEST) update
	cd "$(ZEPHYR_WORKSPACE)" && $(WEST) manifest --validate
	touch "$@"

# Treat the state file as a cache. On a cache miss, stable patch IDs must show
# that the checkout contains an exact ordered prefix of the requested stack.
$(ZEPHYR_PATCH_STATE): $(ZEPHYR_WEST_UPDATE_STAMP) $(ZEPHYR_PATCHES) zephyr-patch-state-force
	@set -e; \
	state="$@"; \
	state_check_dir=$$(mktemp -d "$$state.check.XXXXXX"); \
	trap 'rm -rf "$$state_check_dir"' EXIT; \
	desired_patches="$$state_check_dir/desired-patches"; \
	recorded_patches="$$state_check_dir/recorded-patches"; \
	desired_ids="$$state_check_dir/desired-ids"; \
	applied_ids="$$state_check_dir/applied-ids"; \
	known_ids="$$state_check_dir/known-ids"; \
	: > "$$desired_patches"; \
	: > "$$desired_ids"; \
	for patch in $(ZEPHYR_PATCHES); do \
		sha256sum "$$patch" >> "$$desired_patches"; \
		git patch-id --stable < "$$patch" | sed 's/ .*//' >> "$$desired_ids"; \
	done; \
	sed '/^HEAD /d' "$$state" 2>/dev/null > "$$recorded_patches" || true; \
	current_head=$$(git -C "$(ZEPHYR_WORKSPACE)/zephyr" rev-parse HEAD); \
	recorded_head=$$(sed -n 's/^HEAD //p' "$$state" 2>/dev/null | tail -n 1); \
	if [ "$$recorded_head" = "$$current_head" ] && \
	   cmp -s "$$desired_patches" "$$recorded_patches"; then \
		touch "$$state"; \
		exit 0; \
	fi; \
	git -C "$(ZEPHYR_WORKSPACE)/zephyr" log --reverse --format=%H \
		"$(ZEPHYR_PATCH_BASE)..HEAD" | while read commit; do \
		git -C "$(ZEPHYR_WORKSPACE)/zephyr" show --format= "$$commit" | \
			git patch-id --stable | sed 's/ .*//'; \
	done > "$$applied_ids"; \
	applied_count=$$(wc -l < "$$applied_ids"); \
	desired_count=$$(wc -l < "$$desired_ids"); \
	head -n "$$applied_count" "$$desired_ids" > "$$state_check_dir/desired-prefix"; \
	if [ "$$applied_count" -gt "$$desired_count" ] || \
	   ! cmp -s "$$applied_ids" "$$state_check_dir/desired-prefix"; then \
		printf '%s\n' \
			'Zephyr patch stack was removed, changed, or reordered after being applied.' \
			'Reset the Zephyr workspace before rebuilding this patch stack.' >&2; \
		exit 1; \
	fi; \
	: > "$$known_ids"; \
	git -C "$(ZEPHYR_WORKSPACE)/zephyr" log --format=%H --max-count=256 | while read commit; do \
		git -C "$(ZEPHYR_WORKSPACE)/zephyr" show --format= "$$commit" | git patch-id --stable | sed 's/ .*//' >> "$$known_ids"; \
	done; \
	if [ -n "$(ZEPHYR_PATCHES)" ]; then \
		for patch in $(ZEPHYR_PATCHES); do \
			patch_id=$$(git patch-id --stable < "$$patch" | sed 's/ .*//'); \
			if grep -Fxq "$$patch_id" "$$known_ids" || git -C "$(ZEPHYR_WORKSPACE)/zephyr" apply --reverse --check "$$patch" >/dev/null 2>&1; then \
				continue; \
			fi; \
			git -C "$(ZEPHYR_WORKSPACE)/zephyr" am --whitespace=nowarn "$$patch"; \
		done; \
	fi; \
	state_tmp="$$state_check_dir/state"; \
	cp "$$desired_patches" "$$state_tmp"; \
	printf 'HEAD %s\n' "$$(git -C "$(ZEPHYR_WORKSPACE)/zephyr" rev-parse HEAD)" >> "$$state_tmp"; \
	mv "$$state_tmp" "$$state"; \
	touch "$@"

.PHONY: zephyr-patches-update
zephyr-patches-update: $(ZEPHYR_WEST_UPDATE_STAMP)
	@set -e; \
	zephyr_tree="$(ZEPHYR_WORKSPACE)/zephyr"; \
	patch_dir="$(ZEPHYR_PATCH_DIR)"; \
	patch_state="$(ZEPHYR_PATCH_STATE)"; \
	tmp_dir=$$(mktemp -d); \
	trap 'rm -rf "$$tmp_dir"' EXIT; \
	if ! git -C "$$zephyr_tree" diff --quiet || ! git -C "$$zephyr_tree" diff --cached --quiet; then \
		printf 'Zephyr tree has uncommitted changes; commit or stash them before exporting patches\n' >&2; \
		exit 1; \
	fi; \
	mkdir -p "$$patch_dir" "$$(dirname "$$patch_state")"; \
	git -C "$$zephyr_tree" format-patch --no-stat --no-signature --no-numbered \
		--output-directory "$$tmp_dir" "$(ZEPHYR_PATCH_BASE)..HEAD"; \
	for patch in "$$tmp_dir"/*.patch; do \
		[ -e "$$patch" ] || continue; \
		sed -i '1s/^From [0-9a-f]\{40\} /From 0000000000000000000000000000000000000000 /' "$$patch"; \
	done; \
	find "$$patch_dir" -maxdepth 1 -name '*.patch' -delete; \
	for patch in "$$tmp_dir"/*.patch; do \
		[ -e "$$patch" ] || continue; \
		cp "$$patch" "$$patch_dir/"; \
	done; \
	: > "$$patch_state"; \
	for patch in "$$patch_dir"/*.patch; do \
		[ -e "$$patch" ] || continue; \
		sha256sum "$$patch" >> "$$patch_state"; \
	done; \
	printf 'HEAD %s\n' "$$(git -C "$$zephyr_tree" rev-parse HEAD)" >> "$$patch_state"; \
	touch "$$patch_state"

.PHONY: zephyr-sdk-arm
zephyr-sdk-arm:
	@if ! test -f "$(ZEPHYR_SDK_CMAKE_CONFIG)" || ! test -x "$(ZEPHYR_SDK_ARM_GCC)"; then \
		$(MAKE) zephyr-sdk-arm-install; \
	fi

$(ZEPHYR_SDK_DOWNLOAD_DIR):
	mkdir -p "$@"

$(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_MINIMAL_ARCHIVE): | $(ZEPHYR_SDK_DOWNLOAD_DIR)
	curl -fL --retry 3 --continue-at - "$(ZEPHYR_SDK_RELEASE_URL)/$(ZEPHYR_SDK_MINIMAL_ARCHIVE)" -o "$@"

$(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_ARM_ARCHIVE): | $(ZEPHYR_SDK_DOWNLOAD_DIR)
	curl -fL --retry 3 --continue-at - "$(ZEPHYR_SDK_RELEASE_URL)/$(ZEPHYR_SDK_ARM_ARCHIVE)" -o "$@"

.PHONY: zephyr-sdk-arm-install
zephyr-sdk-arm-install: \
		$(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_MINIMAL_ARCHIVE) \
		$(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_ARM_ARCHIVE)
	mkdir -p "$(dir $(ZEPHYR_SDK_DIR))"
	tar -xJf "$(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_MINIMAL_ARCHIVE)" -C "$(dir $(ZEPHYR_SDK_DIR))"
	mkdir -p "$(ZEPHYR_SDK_GNU_DIR)"
	tar -xJf "$(ZEPHYR_SDK_DOWNLOAD_DIR)/$(ZEPHYR_SDK_ARM_ARCHIVE)" -C "$(ZEPHYR_SDK_GNU_DIR)"
	cd "$(ZEPHYR_SDK_DIR)" && ./setup.sh -h
	touch "$(ZEPHYR_SDK_STAMP)"

.PHONY: app-microbit
app-microbit: zephyr-workspace zephyr-sdk-arm sc26re-build-conf
	cd "$(ZEPHYR_WORKSPACE)" && $(ZEPHYR_BUILD_ENV) $(WEST) build -p "$(ZEPHYR_PRISTINE)" \
		-b bbc_microbit_v2 "$(ZEPHYR_APP)" -d "$(ZEPHYR_APP_BUILD_PREFIX)-microbit" \
		-- -DBOARD_ROOT="$(ZEPHYR_APP)" -DEXTRA_CONF_FILE="$(MICROBIT_CONF_FILES)"

.PHONY: app-microbit-flash
app-microbit-flash: app-microbit
	$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" \
		-c 'init; program $(MICROBIT_APP_HEX) verify reset exit'

.PHONY: app-ibex
app-ibex: zephyr-workspace zephyr-sdk-arm sc26re-build-conf
	cd "$(ZEPHYR_WORKSPACE)" && $(ZEPHYR_BUILD_ENV) $(WEST) build -p "$(ZEPHYR_PRISTINE)" \
		-b steam_controller_ibex/nrf52833 "$(ZEPHYR_APP)" -d "$(ZEPHYR_APP_BUILD_PREFIX)-ibex" \
		-- -DBOARD_ROOT="$(ZEPHYR_APP)" -DEXTRA_CONF_FILE="$(IBEX_CONF_FILES)"

.PHONY: sc26re-build-conf
sc26re-build-conf:
	@mkdir -p "$(ZEPHYR_WORKSPACE)"
	@printf 'CONFIG_IBEX_BUILD_TIMESTAMP=0x%s\n' "$(IBEX_BUILD_TIMESTAMP)" > "$(SC26RE_BUILD_CONF)"

.PHONY: app-ibex-bl-flash
app-ibex-bl-flash: app-ibex
	@test -n "$(IBEX_BL_PORT)" || { echo 'set IBEX_BL_PORT=/dev/ttyACM<number>' >&2; exit 2; }
	./flash.py bl-flash --target ibex --port "$(IBEX_BL_PORT)" --firmware "$(IBEX_CUSTOM_FW)"

.PHONY: app-ibex-flash
app-ibex-flash: app-ibex
	@test -n "$(IBEX_SERIAL)" || { echo 'set IBEX_SERIAL=<controller serial>' >&2; exit 2; }
	./flash.py update --serial "$(IBEX_SERIAL)" --target ibex --ibex-fw "$(IBEX_CUSTOM_FW)"

.PHONY: app-clean
app-clean: app-clean-microbit app-clean-ibex

.PHONY: app-clean-microbit
app-clean-microbit:
	rm -rf "$(ZEPHYR_WORKSPACE)/$(ZEPHYR_APP_BUILD_PREFIX)-microbit"

.PHONY: app-clean-ibex
app-clean-ibex:
	rm -rf "$(ZEPHYR_WORKSPACE)/$(ZEPHYR_APP_BUILD_PREFIX)-ibex"

.PHONY: help
help:
	@printf '%s\n' \
		'App targets:' \
		'make app-microbit                   build controller app for BBC micro:bit v2' \
		'make app-microbit-flash             build, flash, verify, and run the micro:bit app' \
		'make app-ibex                       build Ibex app' \
		'make app-ibex BLE=0                 build Ibex app without Bluetooth' \
		'make app-ibex CDC_DIAG=1            build Ibex app with USB CDC only, no USB HID' \
		'make app-ibex-flash IBEX_SERIAL=... build and flash a stock-runtime genuine Ibex' \
		'make app-ibex-bl-flash IBEX_BL_PORT=/dev/ttyACM0  build and flash genuine Ibex' \
		'make PRISTINE=1 app-ibex            force a pristine Ibex Zephyr rebuild' \
		'make app-clean                      remove micro:bit and Ibex app build directories' \
		'make app-clean-microbit             remove the micro:bit app build directory' \
		'make app-clean-ibex                 remove the Ibex app build directory' \
		'' \
		'App maintenance:' \
		'make format                         clang-format app C sources in place' \
		'make format-check                   verify app C sources match app/.clang-format' \
		'make test                           run host-side unit tests' \
		'make zephyr-sdk-arm                 install pinned Zephyr SDK metadata and ARM toolchain under sdk/' \
		'' \
		'Firmware/source fetching:' \
		'make firmware                       fetch Steam client hardware packages, copy firmware, verify sums' \
		'make updater                        fetch Steam client hardware packages, copy updater cfg/x86_64' \
		'make refresh-steam-client-manifest  update the pinned Steam client manifest' \
		'make clean-firmware                 remove copied firmware files' \
		'' \
		'Bootstub and target utilities:' \
		'make bootstub                       build micro:bit v2 Ibex relay HEX/BIN' \
		'make bootstub-flash                 flash relay+Ibex image with OpenOCD, then reset' \
		'make bootstub-flash-halt            flash relay+Ibex image with OpenOCD, then halt' \
		'make bootstub-debug-server          start OpenOCD with reset vector catch' \
		'make bootstub-run                   reset and run the micro:bit target' \
		'make bootstub-rtt-server            run target and expose RTT channel 0 on TCP 19021' \
		'make microbit-uicr-read             read relevant micro:bit nRF52 UICR words' \
		'make microbit-uicr-provision        write Ibex UICR marker/customer board id' \
		'make microbit-uicr-erase            erase the whole nRF52 UICR area' \
		'make stock-dump                     dump stock flash/FICR/UICR and target metadata via Tigard' \
		'make stock-dump-{flash,ficr,uicr}   dump one persistent stock memory region via Tigard' \
		'make stock-runtime-dump             snapshot RAM/registers from running stock firmware via Tigard'
