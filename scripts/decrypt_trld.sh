#!/usr/bin/env bash
# Rewrite 天锐绿盾 (TRLD Enc*) files to on-disk plaintext.
#
# Official `TRLD --decrypt` is often blocked by policy. This script reads
# through the transparent-decrypt driver, then replaces each file with
# /bin/mv so the write is not re-encrypted.
#
#   ./scripts/decrypt_trld.sh              # repo root
#   ./scripts/decrypt_trld.sh -n           # dry run (list only)
#   ./scripts/decrypt_trld.sh kine types   # only these paths
#
# Saving .hpp/.cpp/.h/.c/.py in Cursor will likely encrypt them again.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DECRYPT_TRLD_SCRIPT_DIR="${SCRIPT_DIR}"

exec python3 - "$@" <<'PY'
from __future__ import annotations

import argparse
import os
import shutil
import stat
import subprocess
import sys
from pathlib import Path

SKIP_DIR_NAMES = {".git", ".cache", "__pycache__"}
ENC_MAGIC = {b"\x84}", b"\x87}", b"\x88}", b"\x89}"}
TMP_SUFFIX = ".plaintext_tmp"


def find_trld() -> str | None:
    for cand in (shutil.which("TRLD"), "/usr/bin/TRLD"):
        if cand and Path(cand).exists():
            return cand
    return None


def is_tmp_name(name: str) -> bool:
    return TMP_SUFFIX in name


def parse_trld_enc_lines(output: str, scan: Path) -> list[Path]:
    found: list[Path] = []
    base = scan if scan.is_dir() else scan.parent
    for line in output.splitlines():
        if "TR_ENC" not in line:
            continue
        path_s = line.split("|")[-1].strip()
        if not path_s:
            continue
        p = Path(path_s)
        if not p.is_absolute():
            p = base / path_s
        found.append(p)
    return found


def list_enc_trld(trld: str, scan: Path) -> list[Path]:
    cmd = [trld, "--file", str(scan)]
    if scan.is_dir():
        cmd.append("-r")
    out = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)
    return parse_trld_enc_lines(out, scan)


def file_brief(path: Path) -> str:
    try:
        return subprocess.check_output(
            ["file", "--brief", str(path)],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def raw_prefix(path: Path, n: int = 2) -> bytes:
    """On-disk bytes via dd (not in the TRLD process whitelist)."""
    try:
        return subprocess.check_output(
            ["/bin/dd", f"if={path}", "bs=1", f"count={n}", "status=none"],
            stderr=subprocess.DEVNULL,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return b""


def is_enc_on_disk(path: Path) -> bool:
    brief = file_brief(path)
    if "Trld Enc" in brief:
        return True
    return raw_prefix(path) in ENC_MAGIC


def iter_files(scan: Path):
    if scan.is_file():
        yield scan
        return
    for dirpath, dirnames, filenames in os.walk(scan):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIR_NAMES]
        for name in filenames:
            if is_tmp_name(name):
                continue
            yield Path(dirpath) / name


def list_enc_fallback(scan: Path) -> list[Path]:
    return [p for p in iter_files(scan) if p.is_file() and is_enc_on_disk(p)]


def collect_targets(paths: list[Path], trld: str | None) -> list[Path]:
    seen: set[Path] = set()
    out: list[Path] = []
    for scan in paths:
        scan = scan.resolve()
        if not scan.exists():
            print(f"skip missing: {scan}", file=sys.stderr)
            continue
        if trld:
            try:
                chunk = list_enc_trld(trld, scan)
            except subprocess.CalledProcessError as e:
                print(
                    f"TRLD --file failed on {scan}, fallback to file(1): {e}",
                    file=sys.stderr,
                )
                chunk = list_enc_fallback(scan)
        else:
            chunk = list_enc_fallback(scan)
        for p in chunk:
            try:
                rp = p.resolve()
            except OSError:
                continue
            if rp in seen or not p.is_file():
                continue
            if any(part in SKIP_DIR_NAMES for part in rp.parts):
                continue
            if is_tmp_name(p.name):
                continue
            seen.add(rp)
            out.append(p)
    return out


def decrypt_one(src: Path) -> None:
    data = src.read_bytes()
    if data[:2] in ENC_MAGIC:
        raise RuntimeError(
            "python still sees ciphertext (this process is not allowed to decrypt)"
        )
    st = src.stat()
    tmp = src.with_name(src.name + TMP_SUFFIX)
    n = 0
    while tmp.exists():
        n += 1
        tmp = src.with_name(f"{src.name}{TMP_SUFFIX}{n}")
    tmp.write_bytes(data)
    try:
        os.chmod(tmp, stat.S_IMODE(st.st_mode))
        subprocess.check_call(["/bin/mv", "-f", str(tmp), str(src)])
    except Exception:
        if tmp.exists():
            tmp.unlink()
        raise
    try:
        os.utime(src, (st.st_atime, st.st_mtime))
    except OSError:
        pass
    if is_enc_on_disk(src):
        raise RuntimeError("still Enc* on disk after rewrite")


def main() -> int:
    script_dir = Path(os.environ["DECRYPT_TRLD_SCRIPT_DIR"]).resolve()
    default_root = script_dir.parent

    ap = argparse.ArgumentParser(
        prog="decrypt_trld.sh",
        description="Decrypt 天锐绿盾 Enc* files in-place to on-disk plaintext.",
    )
    ap.add_argument(
        "paths",
        nargs="*",
        type=Path,
        help="files or directories (default: repository root)",
    )
    ap.add_argument(
        "-n",
        "--dry-run",
        action="store_true",
        help="list encrypted files only",
    )
    args = ap.parse_args()

    roots = [p.expanduser().resolve() for p in args.paths] if args.paths else [default_root]
    trld = find_trld()
    if not trld:
        print("TRLD client not found; detecting Enc* via file(1)/magic.", file=sys.stderr)

    targets = collect_targets(roots, trld)
    if not targets:
        print("No Enc* files found.")
        return 0

    print(f"found {len(targets)} encrypted file(s)")
    ok = 0
    failed: list[tuple[Path, str]] = []
    for p in targets:
        rel: Path | str = p
        try:
            rel = p.relative_to(default_root)
        except ValueError:
            pass
        if args.dry_run:
            print(f"  ENC  {rel}")
            ok += 1
            continue
        try:
            decrypt_one(p)
            print(f"  OK   {rel}  ({p.stat().st_size} bytes)")
            ok += 1
        except Exception as e:
            print(f"  FAIL {rel}: {e}", file=sys.stderr)
            failed.append((p, str(e)))

    print(f"\n{'would decrypt' if args.dry_run else 'decrypted'}: {ok}")
    if failed:
        print(f"failed: {len(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
PY
