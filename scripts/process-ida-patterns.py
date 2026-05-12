#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
import argparse
from collections import Counter
from pathlib import Path
import sys


def parse_pat_row(line):
    parts = line.split()
    for idx, part in enumerate(parts):
        if part.startswith(":"):
            if idx + 1 >= len(parts):
                return None
            name = parts[idx + 1]
            return tuple(parts[:idx]), name, tuple(parts[idx + 2 :])
    return None


def pat_collision_key(line):
    parts = line.split()
    for idx, part in enumerate(parts):
        if part.startswith(":"):
            if idx + 1 >= len(parts) or idx < 4:
                return None
            name = parts[idx + 1]
            return name, parts[idx - 3], parts[idx - 2], parts[0]
    return None


def exc_collision_key(candidate):
    name, signature, _ = candidate
    parts = signature.split()
    if len(parts) < 3:
        return None
    return name, parts[0], parts[1], parts[2]


def normalize_patterns(inputs, out):
    seen = set()
    total = 0
    kept = 0
    duplicate = 0
    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.parent.mkdir(parents=True, exist_ok=True)

    with open(tmp, "w") as outf:
        for path in inputs:
            with open(path, "r", errors="replace") as inf:
                for raw_line in inf:
                    line = raw_line.rstrip("\n")
                    parsed = parse_pat_row(line)
                    if parsed is None:
                        if line.strip():
                            outf.write(line + "\n")
                        continue

                    total += 1
                    key = parsed
                    if key in seen:
                        duplicate += 1
                        continue
                    seen.add(key)
                    kept += 1
                    outf.write(line + "\n")

    tmp.replace(out)
    print(
        f"wrote {out}: kept {kept}/{total} pattern rows " f"({duplicate} exact duplicates removed)",
        flush=True,
    )


def read_collision_groups(exc):
    groups = []
    current = []
    with open(exc, "r", errors="replace") as f:
        for raw_line in f:
            line = raw_line.rstrip("\n")
            if not line.strip():
                if current:
                    groups.append(current)
                    current = []
                continue
            current.append(line)
    if current:
        groups.append(current)
    return groups


def parse_exc_candidate(line):
    if line.startswith(";"):
        return None
    parts = line.split()
    if not parts:
        return None
    return parts[0], " ".join(parts[1:]), line


def prepare_exclusions(exc, mode):
    groups = read_collision_groups(exc)
    kept_duplicate_groups = 0
    excluded_rows = 0
    tmp = exc.with_suffix(exc.suffix + ".tmp")

    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        for group in groups:
            candidates = [parse_exc_candidate(line) for line in group]
            candidates = [candidate for candidate in candidates if candidate is not None]
            if not candidates:
                continue

            names = {name for name, _, _ in candidates}
            if mode == "keep-duplicate-names" and len(names) == 1 and len(candidates) > 1:
                rows_to_exclude = candidates[1:]
                kept_duplicate_groups += 1
            else:
                rows_to_exclude = candidates

            if not rows_to_exclude:
                continue

            for _, _, line in rows_to_exclude:
                f.write(line + "\n")
                excluded_rows += 1
            f.write("\n")

    tmp.replace(exc)
    print(
        f"prepared {exc}: mode {mode}, excluded {excluded_rows} rows, "
        f"kept one representative in {kept_duplicate_groups} duplicate-name groups",
        flush=True,
    )


def filter_patterns(pat, exc, out, mode):
    groups = read_collision_groups(exc)
    removals = Counter()
    kept_duplicate_groups = 0
    ambiguous_groups = 0

    for group in groups:
        candidates = [parse_exc_candidate(line) for line in group]
        candidates = [candidate for candidate in candidates if candidate is not None]
        if not candidates:
            continue

        names = {name for name, _, _ in candidates}
        if mode == "keep-duplicate-names" and len(names) == 1 and len(candidates) > 1:
            rows_to_remove = candidates[1:]
            kept_duplicate_groups += 1
        else:
            rows_to_remove = candidates
            if len(names) > 1:
                ambiguous_groups += 1

        for candidate in rows_to_remove:
            key = exc_collision_key(candidate)
            if key is not None:
                removals[key] += 1

    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.parent.mkdir(parents=True, exist_ok=True)
    removed = 0
    total_rows = 0
    kept_rows = 0
    with open(pat, "r", encoding="utf-8", errors="replace") as inf, open(
        tmp, "w", encoding="utf-8", newline="\n"
    ) as outf:
        for raw_line in inf:
            line = raw_line.rstrip("\n")
            key = pat_collision_key(line)
            if key is None:
                outf.write(line + "\n")
                continue

            total_rows += 1
            if removals[key] > 0:
                removals[key] -= 1
                removed += 1
                continue

            kept_rows += 1
            outf.write(line + "\n")

    missing = sum(removals.values())
    if missing:
        raise RuntimeError(f"failed to match {missing} collision rows in {pat}")

    tmp.replace(out)
    print(
        f"wrote {out}: kept {kept_rows}/{total_rows} pattern rows, "
        f"removed {removed} collision rows "
        f"({kept_duplicate_groups} same-name groups kept, "
        f"{ambiguous_groups} ambiguous-name groups removed)",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Process IDA FLIRT pattern artifacts.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    normalize = subparsers.add_parser(
        "normalize",
        description="Merge PAT files and remove exact duplicate pattern rows.",
    )
    normalize.add_argument("--out", required=True, type=Path)
    normalize.add_argument("inputs", nargs="+", type=Path)

    exclusions = subparsers.add_parser(
        "prepare-exclusions",
        description="Rewrite a sigmake exception file for automatic resolution.",
    )
    exclusions.add_argument("--exc", required=True, type=Path)
    exclusions.add_argument(
        "--mode",
        choices=("keep-duplicate-names", "exclude-all"),
        default="keep-duplicate-names",
    )

    filter_pat = subparsers.add_parser(
        "filter-pat",
        description="Filter collision rows directly from a PAT file.",
    )
    filter_pat.add_argument("--pat", required=True, type=Path)
    filter_pat.add_argument("--exc", required=True, type=Path)
    filter_pat.add_argument("--out", required=True, type=Path)
    filter_pat.add_argument(
        "--mode",
        choices=("keep-duplicate-names", "exclude-all"),
        default="keep-duplicate-names",
    )

    args = parser.parse_args()
    if args.command == "normalize":
        normalize_patterns(args.inputs, args.out)
    elif args.command == "prepare-exclusions":
        prepare_exclusions(args.exc, args.mode)
    elif args.command == "filter-pat":
        filter_patterns(args.pat, args.exc, args.out, args.mode)
    else:
        parser.error(f"unknown command: {args.command}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
