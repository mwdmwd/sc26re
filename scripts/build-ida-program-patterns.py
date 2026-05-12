#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
import argparse
import fnmatch
import os
from pathlib import Path
import re
import shutil
import sys


def read_ignored_symbols(path):
    patterns = []
    exact = set()
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("/") and line.endswith("/") and len(line) > 1:
                patterns.append(re.compile(line[1:-1]))
            elif "*" in line or "?" in line:
                patterns.append(re.compile(fnmatch.translate(line)))
            else:
                exact.add(line)
    return exact, patterns


def ignored_name(name, exact, patterns):
    if name in exact:
        return True
    return any(pattern.fullmatch(name) for pattern in patterns)


def prepare_idausr(dst, source):
    dst.mkdir(parents=True, exist_ok=True)
    reg = source / "ida.reg"
    if not reg.exists():
        raise RuntimeError(f"missing IDA registry/license state: {reg}")
    shutil.copy2(reg, dst / "ida.reg")

    cfg = source / "cfg"
    if cfg.is_dir():
        shutil.copytree(cfg, dst / "cfg", dirs_exist_ok=True)


def safe_work_name(path, root):
    try:
        rel = path.resolve().relative_to(root.resolve())
    except ValueError:
        rel = Path(path.name)
    return "__".join(rel.with_suffix("").parts) + path.suffix


def main():
    parser = argparse.ArgumentParser(
        description="Build IDA Makesig pattern rows from linked corpus ELFs."
    )
    parser.add_argument("--program-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--ignored-symbols", required=True)
    parser.add_argument("--idausr-source", default=str(Path.home() / ".idapro"))
    parser.add_argument("--idausr", required=True)
    parser.add_argument("--idadir", default="/opt/ida-pro")
    args = parser.parse_args()

    program_dir = Path(args.program_dir)
    out = Path(args.out)
    work_dir = Path(args.work_dir)
    idausr = Path(args.idausr)

    inputs = sorted(program_dir.rglob("*.elf"))
    if not inputs:
        raise RuntimeError(f"no seed ELFs found under {program_dir}")

    prepare_idausr(idausr, Path(args.idausr_source))
    os.environ["IDAUSR"] = str(idausr)
    os.environ["IDADIR"] = args.idadir

    import idapro
    import ida_funcs  # pyright: ignore[reportMissingModuleSource]
    import ida_name  # pyright: ignore[reportMissingModuleSource]
    import idautils  # pyright: ignore[reportMissingModuleSource]

    exact, patterns = read_ignored_symbols(args.ignored_symbols)
    work_dir.mkdir(parents=True, exist_ok=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp_out = out.with_suffix(out.suffix + ".tmp")
    tmp_out.unlink(missing_ok=True)

    total_patterns = 0
    total_functions = 0
    total_ignored = 0

    with open(tmp_out, "wb") as combined:
        for src in inputs:
            work_name = safe_work_name(src, program_dir)
            work_input = work_dir / work_name
            if work_input.exists() or work_input.is_symlink():
                work_input.unlink()
            shutil.copy2(src, work_input)

            log_path = work_input.with_suffix(work_input.suffix + ".log")
            rc = idapro.open_database(
                str(work_input),
                True,
                args=f"-c -L{log_path}",
            )
            if rc != 0:
                raise RuntimeError(f"IDA failed to open {src}: {rc}")

            funcs = list(idautils.Functions())
            ignored_functions = 0
            ignored_names = 0
            function_starts = set(funcs)
            for ea, name in list(idautils.Names()):
                if not ignored_name(name, exact, patterns):
                    continue
                if ea in function_starts:
                    if ida_name.set_name(
                        ea,
                        f"sub_{ea:X}",
                        ida_name.SN_FORCE | ida_name.SN_NOWARN | ida_name.SN_AUTO,
                    ):
                        ignored_functions += 1
                        ignored_names += 1
                elif ida_name.set_name(ea, "", ida_name.SN_NOWARN):
                    ignored_names += 1

            # Some function names are not present in Names() until requested.
            for ea in funcs:
                name = ida_funcs.get_func_name(ea) or ida_name.get_name(ea)
                if ignored_name(name, exact, patterns):
                    if ida_name.set_name(
                        ea,
                        f"sub_{ea:X}",
                        ida_name.SN_FORCE | ida_name.SN_NOWARN | ida_name.SN_AUTO,
                    ):
                        ignored_functions += 1
                        ignored_names += 1

            ok = idapro.make_signatures(True)
            idapro.close_database(False)
            if not ok:
                raise RuntimeError(f"IDA failed to create pattern file for {src}")

            pat = work_input.with_suffix(work_input.suffix + ".pat")
            if not pat.exists():
                raise RuntimeError(f"IDA did not write expected pattern file: {pat}")
            data = pat.read_bytes()
            if data and not data.endswith(b"\n"):
                data += b"\n"
            combined.write(data)

            pattern_count = sum(1 for line in data.splitlines() if line.strip())
            total_patterns += pattern_count
            total_functions += len(funcs)
            total_ignored += ignored_names
            print(
                f"{src}: {pattern_count} patterns, "
                f"{len(funcs)} functions, ignored "
                f"{ignored_functions} functions/{ignored_names} names",
                flush=True,
            )

    tmp_out.replace(out)
    print(
        f"wrote {out}: {total_patterns} patterns from "
        f"{len(inputs)} ELFs ({total_functions} functions, ignored {total_ignored} names)",
        flush=True,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
