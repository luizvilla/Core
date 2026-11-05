#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file make_record.py
## @brief Writes the test record (record.md) of a campaign from the criterion result lines.
## @see tests/common/README.md, docs/testing/test-procedure.md §8
"""Build the test record (record.md) of a campaign from the criterion lines.

Usage: make_record.py --out DIR --levels L0,L1 LOG [LOG ...]

Each LOG is the captured output of one test binary or script. Lines that start
with "@@<TAB>" are criteria (see tests/common/check.h); everything else is
ignored here and stays available in the log file.

Exit code: 1 if any criterion is FAIL, 0 otherwise. Standard library only.
"""

from __future__ import annotations

import argparse
import os
import datetime as dt
import platform
import re
import subprocess
import sys
from collections import Counter, OrderedDict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
# Same override as tests/common/l0_build.sh, so the record stamps the checkout under test.
LIBRARY = Path(os.environ.get("SINGLE_PHASE_INVERTER_DIR",
                              REPO / "owntech/lib/USB/single_phase_inverter")).resolve()
LIBRARY_OVERRIDDEN = "SINGLE_PHASE_INVERTER_DIR" in os.environ
PROCEDURE = REPO / "docs/testing/test-procedure.md"
ORDER = {"BLOCKED": 0, "FAIL": 1, "PASS": 2, "INFO": 2}


def run(cmd: list[str], cwd: Path = REPO) -> str:
    try:
        return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def git_state(path: Path) -> str:
    sha = run(["git", "rev-parse", "--short=7", "HEAD"], path) or "unknown"
    dirty = run(["git", "status", "--porcelain", "--untracked-files=no"], path)
    return sha + ("-dirty" if dirty else "")


def library_source(path: Path) -> str:
    """Where the library checkout comes from: remote repository and branch."""
    url = run(["git", "remote", "get-url", "origin"], path)
    repo = re.sub(r"(\.git)?$", "", re.sub(r"^.*github\.com[:/]", "", url)) if url else "unknown remote"
    branch = run(["git", "branch", "--show-current"], path) or "detached"
    return f"{repo} `{branch}`"


def contract_version() -> str:
    """Contract version from the procedure's §3 heading, e.g. "v1"."""
    match = re.search(r"## 3\. Model .* contract \((v\d+)\)", PROCEDURE.read_text(encoding="utf-8"))
    return match.group(1) if match else "unknown"


def procedure_version() -> str:
    match = re.search(r"Procedure version \| ([^|]+)\|", PROCEDURE.read_text(encoding="utf-8"))
    return match.group(1).strip() if match else "unknown"


def pulsim_version() -> str:
    for python in (REPO / ".venv/bin/python", Path(sys.executable)):
        if python.exists():
            out = run([str(python), "-c", "import pulsim; print(pulsim.__version__)"])
            if out:
                return out
    return "not installed"


def parse(logs: list[Path]) -> list[dict]:
    rows = []
    for log in logs:
        for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
            if not line.startswith("@@\t"):
                continue
            parts = line.split("\t")[1:]
            parts += [""] * (5 - len(parts))
            rid, verdict, desc, limit, measured = parts[:5]
            rows.append(dict(id=rid, verdict=verdict, desc=desc, limit=limit,
                             measured=measured, log=log.name))
    return rows


def findings_register() -> list[dict]:
    """Findings from the procedure's findings register (§11).

    Each row of the register table whose first cell links an issue (#N)
    becomes {issue, url, criteria, status, open}. "criteria" holds the IDs of
    the second cell; a case ID (L0-GFL-08) covers all its criteria
    (L0-GFL-08.a, ...). Rows whose status starts with "Closed" are not open: a
    FAIL on their criteria is a regression and is reported as new.
    """
    text = PROCEDURE.read_text(encoding="utf-8")
    start = text.find("## 11. Findings register")
    if start < 0:
        return []
    end = text.find("\n## ", start + 1)
    findings = []
    for line in text[start:end if end > 0 else None].splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 5:
            continue
        issue = re.search(r"#(\d+)", cells[0])
        if not issue:
            continue
        url = re.search(r"\((https?://[^)]+)\)", cells[0])
        findings.append(dict(
            issue=f"#{issue.group(1)}",
            url=url.group(1) if url else "",
            criteria=[c.strip(" `") for c in cells[1].split(",") if c.strip(" `")],
            status=cells[4],
            open=not cells[4].lower().startswith("closed")))
    return findings


def covers(finding: dict, criterion_id: str) -> bool:
    """Whether a finding lists a criterion, directly or through its test case."""
    return any(criterion_id == c or criterion_id.startswith(c + ".") for c in finding["criteria"])


def finding_for(criterion_id: str, findings: list[dict]):
    """Open finding that covers a criterion, or None."""
    return next((f for f in findings if f["open"] and covers(f, criterion_id)), None)


def case_of(criterion_id: str) -> str:
    return criterion_id.split(".")[0]


def sort_key(row: dict) -> tuple:
    return (ORDER.get(row["verdict"], 3), row["id"])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--levels", default="L0,L1")
    parser.add_argument("logs", nargs="*", type=Path)
    args = parser.parse_args()

    rows = parse(args.logs)
    findings = findings_register()
    for r in rows:
        r["finding"] = finding_for(r["id"], findings) if r["verdict"] == "FAIL" else None
    counts = Counter(r["verdict"] for r in rows)
    known = [r for r in rows if r["finding"]]
    new = [r for r in rows if r["verdict"] == "FAIL" and not r["finding"]]
    # Open findings whose criteria all ran without FAIL or BLOCKED: ready to close.
    fixed = []
    for f in findings:
        covered = [r for r in rows if covers(f, r["id"])]
        if f["open"] and covered and not any(r["verdict"] in ("FAIL", "BLOCKED") for r in covered):
            fixed.append(f)
    # A campaign that reports nothing must not look green.
    failed = counts["FAIL"] > 0 or not rows
    now = dt.datetime.now().astimezone()
    core = git_state(REPO)
    library = git_state(LIBRARY) if LIBRARY.exists() else "not fetched"
    gcc = run(["g++", "-dumpfullversion"]) or "unknown"
    levels = [lvl for lvl in args.levels.split(",") if lvl]
    plant = "none"  # set by the L1 runner once a plant model exists

    cases: "OrderedDict[str, list[dict]]" = OrderedDict()
    for row in sorted(rows, key=lambda r: r["id"]):
        cases.setdefault(case_of(row["id"]), []).append(row)

    out = []
    out += ["---",
            f"campaign: {args.out.name}",
            f"date: {now.isoformat(timespec='seconds')}",
            f"operator: {run(['git', 'config', 'user.name']) or 'unknown'}",
            f"procedure_version: {procedure_version()}",
            f"core_commit: {core}",
            f"library_commit: {library}",
            f"library_source: {library_source(LIBRARY).replace('`', '')}",
            f"plant_model: {plant}",
            f"contract_version: {contract_version()}",
            f'toolchain: {{ gcc: "{gcc}", python: "{platform.python_version()}", '
            f'pulsim: "{pulsim_version()}" }}',
            f"levels: [{', '.join(levels)}]",
            f"verdict: {'FAIL' if failed else 'PASS'}",
            f"new_failures: {len(new)}",
            f"known_failures: {len(known)}",
            "known_findings: [" + ", ".join(sorted({r['finding']['issue'] for r in known})) + "]",
            "findings_to_close: [" + ", ".join(f["issue"] for f in fixed) + "]",
            "counts: { " + ", ".join(f"{v}: {counts[v]}" for v in
                                    ("PASS", "FAIL", "INFO", "BLOCKED")) + " }",
            "---", ""]

    out += [f"# Test record {args.out.name}", "",
            f"**Campaign verdict: {'FAIL' if failed else 'PASS'}**: "
            f"{counts['PASS']} pass, {counts['FAIL']} fail, {counts['INFO']} info, "
            f"{counts['BLOCKED']} blocked, over {len(cases)} test cases.", ""]
    if counts["FAIL"]:
        out += [f"> **Failures:** {len(new)} new, {len(known)} known "
                f"(open findings, procedure §11). Only new failures need an issue.", ""]
    if fixed:
        out += ["> **Findings ready to close:** " +
                ", ".join(f"[{f['issue']}]({f['url']})" if f["url"] else f["issue"] for f in fixed) +
                ": all their criteria pass in this campaign. Close the issue quoting this "
                "campaign ID and mark the finding Closed in §11.", ""]
    if counts["BLOCKED"]:
        blocked = sorted({case_of(r["id"]) for r in rows if r["verdict"] == "BLOCKED"})
        out += [f"> **Blocked:** {', '.join(blocked)}. See the results table for the reason.", ""]
    if not rows:
        out += ["> **No criteria were reported.** Check the log files in this folder.", ""]

    out += ["## Setup", "",
            "| Item | Value |", "|---|---|",
            f"| Core commit | `{core}` |",
            f"| Library commit | `{library}` (single_phase_inverter, "
            + library_source(LIBRARY)
            + (f", override `{LIBRARY}`" if LIBRARY_OVERRIDDEN else "") + ") |",
            f"| Plant model | {plant} |",
            f"| Contract | {contract_version()} |",
            "| Control period | 100 µs (cold loop every 10 ticks) |",
            f"| Toolchain | gcc {gcc}, Python {platform.python_version()}, "
            f"Pulsim {pulsim_version()} |",
            f"| Host | {platform.system()} {platform.release()} {platform.machine()} |",
            f"| Levels run | {', '.join(levels)} |", ""]

    if rows:
        out += ["## Verdicts", "", "```mermaid", "pie showData title Criteria by verdict"]
        out += [f'    "{v}" : {counts[v]}' for v in ("PASS", "FAIL", "INFO", "BLOCKED")
                if counts[v]]
        out += ["```", ""]

        out += ["## Test cases", "",
                "| Test case | Criteria | Verdict |", "|---|---|---|"]
        for case, items in cases.items():
            verdicts = {r["verdict"] for r in items}
            worst = next((v for v in ("FAIL", "BLOCKED", "PASS", "INFO") if v in verdicts),
                         "?")
            out.append(f"| {case} | {len(items)} | {worst} |")
        out.append("")

        out += ["## Results", "",
                "| Criterion | Description | Limit | Measured | Verdict |",
                "|---|---|---|---|---|"]
        for r in sorted(rows, key=sort_key):
            verdict = r["verdict"]
            if r["finding"]:
                f = r["finding"]
                link = f"[{f['issue']}]({f['url']})" if f["url"] else f["issue"]
                verdict = f"FAIL (known {link})"
            elif verdict == "FAIL":
                verdict = "FAIL (**new**)"
            out.append(f"| {r['id']} | {r['desc']} | {r['limit']} | {r['measured']} | {verdict} |")
        out.append("")

    out += ["## Raw data", "", "| File | Content |", "|---|---|"]
    out += [f"| `{log.name}` | Full output of `{log.stem}` |" for log in args.logs]
    out.append("")

    if new:
        out += ["## Follow-up", "",
                "New failures: " + ", ".join(r["id"] for r in new) + ".", "",
                "Open one issue per failing test case on `luizvilla/Core` with the "
                "`needs-triage` label, quoting this campaign ID and the criterion IDs, "
                "then add it to the findings register (procedure §8.4, §11). Known "
                "failures already have an issue: comment there only if the behaviour changed.", ""]

    args.out.mkdir(parents=True, exist_ok=True)
    record = args.out / "record.md"
    record.write_text("\n".join(out), encoding="utf-8")

    width = max([len(r["id"]) for r in rows] + [9])
    for r in sorted(rows, key=sort_key):
        tag = (f" (known {r['finding']['issue']})" if r["finding"]
               else " (NEW)" if r["verdict"] == "FAIL" else "")
        print(f"  {r['id']:<{width}}  {r['verdict']:<7}  {r['desc']}{tag}")
    print(f"\n{'FAIL' if failed else 'PASS'}: {counts['PASS']} pass, {counts['FAIL']} fail "
          f"({len(new)} new, {len(known)} known), {counts['INFO']} info, "
          f"{counts['BLOCKED']} blocked")
    for f in fixed:
        print(f"Finding {f['issue']}: all its criteria pass. Ready to close.")
    print(f"Record: {record.relative_to(REPO) if record.is_relative_to(REPO) else record}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
