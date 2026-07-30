#!/usr/bin/env python3
"""Build every OwnTech example from a local ``examples`` checkout against a
local ``Core`` checkout, to check that Core changes don't break them.

The build happens in an isolated copy of the Core tree (under a temp dir, or
--workdir) so the real working copy's uncommitted files (src/main.cpp,
src/app.ini, ...) are never touched.

Usage:
    python3 scripts/build_examples.py
    python3 scripts/build_examples.py --filter twist
    python3 scripts/build_examples.py --examples ../examples --env USB
"""
import argparse
import datetime
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# Only these are actually compiled; README.md and helper scripts (.py/.m/...)
# listed in library.json are documentation/tooling, not build inputs.
BUILD_FILE_EXTENSIONS = {".cpp", ".h", ".hpp", ".ini", ".conf", ".overlay"}

# Matches the committed default Core/src/app.ini. Used for the handful of
# examples (e.g. "Open Loop PWM") that don't ship their own app.ini and rely
# on the project default (Twist shield).
DEFAULT_APP_INI = """# Do not edit or remove the following line
[env]

board_shield = twist
board_shield_version = 1_4_2

lib_deps=
    control_library = https://github.com/owntech-foundation/control_library.git
    owntech_examples = https://github.com/owntech-foundation/examples.git
    scope = https://github.com/owntech-foundation/scopemimicry.git
"""


def resolve_pio(explicit: str | None) -> str:
    """Find a working PlatformIO CLI. The `pio` on PATH can be a broken/old
    system package (seen: PlatformIO 4.3.4 crashing on a click API change),
    while PlatformIO's own installer puts a working one in ~/.platformio."""
    candidates = []
    if explicit:
        candidates.append(explicit)
    candidates.append(str(Path.home() / ".platformio" / "penv" / "bin" / "pio"))
    if shutil.which("pio"):
        candidates.append("pio")

    for candidate in candidates:
        if candidate != "pio" and not Path(candidate).exists():
            continue
        try:
            subprocess.run([candidate, "--version"], capture_output=True, text=True, timeout=15, check=True)
            return candidate
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
            continue

    sys.exit("error: could not find a working PlatformIO CLI (tried: " + ", ".join(candidates) + ")")


def load_examples(examples_dir: Path):
    data = json.loads((examples_dir / "library.json").read_text())
    return data["examples"]


def copy_core_snapshot(core_dir: Path, workdir: Path) -> Path:
    dest = workdir / "core"
    if dest.exists():
        print(f"Reusing existing Core snapshot at {dest} (delete it to force a fresh copy).")
        return dest
    print(f"Copying Core checkout ({core_dir}) -> {dest} ...")
    # .git is irrelevant to the build and .pio is excluded on purpose: it
    # bakes in absolute paths from the original checkout, so a copied .pio
    # would be invalid. The first build in the new location pays the full
    # framework-build cost once; every example after that reuses it.
    shutil.copytree(
        core_dir,
        dest,
        ignore=shutil.ignore_patterns(".git", ".pio"),
        symlinks=True,
    )
    return dest


def stage_example(example: dict, examples_dir: Path, core_src: Path, pristine_src: Path):
    shutil.rmtree(core_src)
    shutil.copytree(pristine_src, core_src, symlinks=True)

    base = examples_dir / example["base"]
    has_app_ini = False
    for rel in example.get("files", []):
        if Path(rel).suffix not in BUILD_FILE_EXTENSIONS:
            continue
        src_file = base / rel
        if not src_file.exists():
            continue
        dest_file = core_src / rel
        dest_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_file, dest_file)
        if rel == "app.ini":
            has_app_ini = True

    if not has_app_ini:
        (core_src / "app.ini").write_text(DEFAULT_APP_INI)


def build_one(core_dir: Path, env: str, log_path: Path, timeout: int, pio: str):
    try:
        proc = subprocess.run(
            [pio, "run", "-e", env],
            cwd=core_dir,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = proc.stdout + proc.stderr
        log_path.write_text(output)
        return proc.returncode == 0, output
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or "") + (exc.stderr or "") + f"\n[timed out after {timeout}s]"
        log_path.write_text(output)
        return False, output


def write_markdown_report(report_path: Path, results: list, core_dir: Path, examples_dir: Path, env: str) -> None:
    passed = [r for r in results if r[2]]
    failed = [r for r in results if not r[2]]

    lines = [
        "# Example Build Report",
        "",
        f"- Generated: {datetime.datetime.now().isoformat(timespec='seconds')}",
        f"- Core: `{core_dir}`",
        f"- Examples: `{examples_dir}`",
        f"- PlatformIO environment: `{env}`",
        f"- Result: **{len(passed)}/{len(results)} passed**",
        "",
        "| Status | Example | Path | Log |",
        "|---|---|---|---|",
    ]
    for name, base, ok, log_path in results:
        status = "✅ PASS" if ok else "❌ FAIL"
        log_rel = log_path.relative_to(report_path.parent)
        lines.append(f"| {status} | {name} | `{base}` | [{log_path.name}]({log_rel}) |")

    if failed:
        lines += ["", "## Failed examples", ""]
        for name, base, _, log_path in failed:
            log_rel = log_path.relative_to(report_path.parent)
            lines.append(f"- **{name}** (`{base}`) — [{log_path.name}]({log_rel})")

    report_path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--core", default=str(Path(__file__).resolve().parent.parent),
                         help="Path to the Core checkout to validate (default: this script's own repo).")
    parser.add_argument("--examples", default=None,
                         help="Path to the examples checkout (default: ../examples next to --core).")
    parser.add_argument("--env", default="USB", help="PlatformIO environment to build (default: USB).")
    parser.add_argument("--filter", default=None,
                         help="Only build examples whose name or base path contains this substring (case-insensitive).")
    parser.add_argument("--workdir", default=None,
                         help="Directory to build in (default: a fresh temp dir, kept after the run).")
    parser.add_argument("--clean", action="store_true", help="Delete the build workdir when done.")
    parser.add_argument("--timeout", type=int, default=1200, help="Per-example build timeout in seconds (default: 1200).")
    parser.add_argument("--pio", default=None, help="Path to the PlatformIO CLI to use (default: auto-detect).")
    args = parser.parse_args()

    core_dir = Path(args.core).resolve()
    examples_dir = Path(args.examples).resolve() if args.examples else (core_dir.parent / "examples")

    if not (core_dir / "platformio.ini").exists():
        sys.exit(f"error: {core_dir} does not look like a Core checkout (no platformio.ini)")
    if not (examples_dir / "library.json").exists():
        sys.exit(f"error: {examples_dir} does not look like the examples repo (no library.json)")
    pio = resolve_pio(args.pio)
    print(f"Using PlatformIO CLI: {pio}")

    examples = load_examples(examples_dir)
    if args.filter:
        needle = args.filter.lower()
        examples = [e for e in examples if needle in e["name"].lower() or needle in e["base"].lower()]
    if not examples:
        sys.exit("No examples matched the given filter.")

    workdir = Path(args.workdir).resolve() if args.workdir else Path(tempfile.mkdtemp(prefix="owntech_examples_build_"))
    workdir.mkdir(parents=True, exist_ok=True)
    logs_dir = workdir / "logs"
    logs_dir.mkdir(exist_ok=True)

    build_core = copy_core_snapshot(core_dir, workdir)
    build_src = build_core / "src"
    pristine_src = workdir / "src.pristine"
    if not pristine_src.exists():
        shutil.copytree(build_src, pristine_src, symlinks=True)

    print(f"\nBuilding {len(examples)} example(s) with `pio run -e {args.env}` in {build_core}\n")

    results = []
    for i, example in enumerate(examples, 1):
        name = example["name"]
        print(f"[{i}/{len(examples)}] {name} ({example['base']}) ... ", end="", flush=True)
        stage_example(example, examples_dir, build_src, pristine_src)
        log_path = logs_dir / f"{i:02d}_{name.replace('/', '_').replace(' ', '_')}.log"
        ok, output = build_one(build_core, args.env, log_path, args.timeout, pio)
        print("OK" if ok else "FAIL")
        if not ok:
            tail = "\n".join(output.splitlines()[-25:])
            print("  --- last lines of build log ---")
            for line in tail.splitlines():
                print(f"  {line}")
            print(f"  (full log: {log_path})")
        results.append((name, example["base"], ok, log_path))

    passed = [r for r in results if r[2]]
    failed = [r for r in results if not r[2]]

    print(f"\n{'=' * 60}")
    print(f"{len(passed)}/{len(results)} examples built successfully.")
    if failed:
        print("\nFailed examples:")
        for name, base, _, log_path in failed:
            print(f"  - {name} ({base}) -> {log_path}")

    report_path = workdir / "report.md"
    write_markdown_report(report_path, results, core_dir, examples_dir, args.env)
    print(f"\nMarkdown report: {report_path}")

    if args.clean:
        shutil.rmtree(workdir)
        print("(--clean was set: workdir, logs, and the report above were deleted)")
    else:
        print(f"Build workdir kept at: {workdir}")

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
