#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later

set -eu

if [ "${IBEX_BUILD_TIMESTAMP:-}" ]; then
	timestamp=${IBEX_BUILD_TIMESTAMP#0x}
	timestamp=${timestamp#0X}
	printf '%s\n' "$timestamp" | tr '[:lower:]' '[:upper:]'
	exit 0
fi

count=
describe=$(git describe --tags --match 'r*' --long --always 2>/dev/null || true)
case "$describe" in
	r-*-g*)
		count=${describe%-g*}
		count=${count##*-}
		;;
esac

if [ -z "$count" ]; then
	count=$(git rev-list --count HEAD 2>/dev/null || printf 0)
fi

printf 'F055%04X\n' "$((count & 0xffff))"
