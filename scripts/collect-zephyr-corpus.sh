#!/usr/bin/env bash
set -eu

workspace=${1:-zephyr}
program_out=${2:-corpus/fid-programs}
lib_out=${3:-corpus/static-libs}
report_out=${4:-corpus/reports}
archive_out=${5:-corpus/fid-archives}

compiler=${ZEPHYR_CORPUS_COMPILER:-gnuarmemb}
family=${ZEPHYR_CORPUS_FAMILY:-steamctl-ncs}
version=${ZEPHYR_CORPUS_VERSION:-v2.9.0-ncs2}

mkdir -p \
	"$program_out/$compiler/$family/$version" \
	"$archive_out/$compiler/$family/$version" \
	"$lib_out" \
	"$report_out"

counts="$report_out/seed-symbol-counts.tsv"
printf 'seed\telf\tdefined_symbols\n' > "$counts"

copy_seed() {
	seed=$1
	rel=$2
	src="$workspace/$rel"

	if [ ! -f "$src" ]; then
		echo "missing seed ELF: $src" >&2
		exit 1
	fi

	dst_dir="$program_out/$compiler/$family/$version/$seed"
	mkdir -p "$dst_dir"
	cp -f "$src" "$dst_dir/$seed.elf"

	count=$(arm-none-eabi-nm --defined-only --format=posix "$src" | wc -l)
	printf '%s\t%s\t%s\n' "$seed" "$src" "$count" >> "$counts"

	app_dir=${src%/zephyr/zephyr.elf}
	lib_dir="$lib_out/$seed"
	archive_dir="$archive_out/$compiler/$family/$version/$seed-archives"
	mkdir -p "$lib_dir" "$archive_dir"
	find "$app_dir" -type f -name '*.a' -print | while IFS= read -r lib; do
		rel_lib=${lib#"$app_dir"/}
		safe=$(printf '%s' "$rel_lib" | sed 's#[/[:space:]]#__#g')
		cp -f "$lib" "$lib_dir/$safe"
		cp -f "$lib" "$archive_dir/$safe"
	done
}

copy_seed hello "build-steamctl-hello/hello_world/zephyr/zephyr.elf"
copy_seed hids "build-steamctl-hids/peripheral_hids/zephyr/zephyr.elf"
copy_seed esb-prx "build-steamctl-esb-prx/esb_prx/zephyr/zephyr.elf"
copy_seed usb-hid "build-steamctl-usb-hid/hid-keyboard/zephyr/zephyr.elf"
copy_seed cdc-acm "build-steamctl-cdc-acm/cdc_acm/zephyr/zephyr.elf"

printf 'wrote %s\n' "$counts"
