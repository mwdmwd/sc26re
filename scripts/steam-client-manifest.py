#!/usr/bin/env python3
import re
import sys


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: steam-client-manifest.py <manifest> <package>...")

    manifest = sys.argv[1]
    wanted = set(sys.argv[2:])
    found = {name: {} for name in wanted}
    stack = []
    pending = None

    with open(manifest) as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line == "{":
                if pending is None:
                    sys.exit("unexpected block opener")
                stack.append(pending)
                pending = None
                continue
            if line == "}":
                if stack:
                    stack.pop()
                pending = None
                continue

            values = re.findall(r'"([^"]*)"', line)
            if len(values) == 1:
                pending = values[0]
                continue
            if len(values) < 2:
                continue

            key, value = values[0], values[1]
            if stack and stack[-1] in found and key in ("file", "sha2"):
                found[stack[-1]][key] = value

    missing = []
    for name in sys.argv[2:]:
        entry = found[name]
        if "file" not in entry or "sha2" not in entry:
            missing.append(name)
            continue
        print(name, entry["file"], entry["sha2"])

    if missing:
        sys.exit("missing package data: " + ", ".join(missing))


if __name__ == "__main__":
    main()
