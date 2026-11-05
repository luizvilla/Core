## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file check.py
## @brief Per-criterion checks for Python test cases; same result lines as check.h.
## @see tests/common/README.md, docs/testing/test-procedure.md §5
"""Per-criterion checks for Python (L1) test cases.

Python twin of check.h: a Criterion collects observations and prints exactly
one result line, read by make_record.py::

    @@<TAB>id<TAB>verdict<TAB>description<TAB>limit<TAB>measured

Usage::

    with Criterion("L1-FWB-01.a", "Block fires every 100 us") as c:
        c.near("fires per second", fires, 10000, 1)
    info("L1-FWB-01.g", "Wall time per simulated second", 0.8, "s")
    sys.exit(finish())
"""

from __future__ import annotations

import math
import sys

_failures = 0


def _clean(text: str) -> str:
    """Makes text safe for one tab-separated cell and a Markdown table cell."""
    return str(text).replace("\t", " ").replace("\n", " ").replace("|", " ")


def num(value: float) -> str:
    """Formats a number with six significant digits."""
    return f"{value:.6g}"


def emit(cid: str, verdict: str, desc: str, limit: str, measured: str) -> None:
    """Prints one result line."""
    print("@@\t" + "\t".join(_clean(x) for x in (cid, verdict, desc, limit, measured)), flush=True)


class Criterion:
    """One pass/fail criterion made of any number of observations.

    Passes when at least one observation was made and none failed. Prints its
    result line when the ``with`` block ends or end() is called.
    """

    def __init__(self, cid: str, desc: str):
        self.cid, self.desc = cid, desc
        self.observations = 0
        self.failed = 0
        self.first_failure = ""
        self.worst_error = 0.0
        self.tol = None
        self.done = False

    def __enter__(self) -> "Criterion":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if exc_type is not None:
            self.check(f"raised {exc_type.__name__}: {exc}", False)
        self.end()
        return False

    def _fail(self, what: str) -> None:
        if self.failed == 0:
            self.first_failure = what
        self.failed += 1

    def check(self, what: str, ok: bool) -> bool:
        """Records a boolean observation."""
        self.observations += 1
        if not ok:
            self._fail(what)
        return bool(ok)

    def near(self, what: str, measured: float, expected: float, tol: float) -> bool:
        """Records |measured - expected| <= tol (NaN fails)."""
        self.observations += 1
        self.tol = tol if self.tol is None else min(self.tol, tol)
        err = abs(measured - expected)
        if not err <= tol:
            self._fail(f"{what} = {num(measured)}, expected {num(expected)} ± {num(tol)}")
            return False
        self.worst_error = max(self.worst_error, err)
        return True

    def range(self, what: str, measured: float, lo: float, hi: float) -> bool:
        """Records lo <= measured <= hi (NaN fails)."""
        self.observations += 1
        if not (lo <= measured <= hi):
            self._fail(f"{what} = {num(measured)}, outside [{num(lo)}, {num(hi)}]")
            return False
        return True

    def end(self) -> None:
        """Prints the result line (once)."""
        global _failures
        if self.done:
            return
        self.done = True
        ok = self.failed == 0 and self.observations > 0
        limit = "all observations hold" if self.tol is None else f"abs error ≤ {num(self.tol)}"
        if self.observations == 0:
            measured = "no observation made"
        elif ok:
            measured = f"{self.observations}/{self.observations} ok"
            if self.tol is not None:
                measured += f", max abs error {num(self.worst_error)}"
        else:
            measured = f"{self.failed}/{self.observations} failed; first: {self.first_failure}"
        if not ok:
            _failures += 1
        emit(self.cid, "PASS" if ok else "FAIL", self.desc, limit, measured)


def info(cid: str, desc: str, value: float, unit: str = "") -> None:
    """Reports a measured value with no stated limit (verdict INFO)."""
    text = "nan" if value is None or (isinstance(value, float) and math.isnan(value)) else num(value)
    emit(cid, "INFO", desc, "—", text + (f" {unit}" if unit else ""))


def blocked(cid: str, desc: str, reason: str) -> None:
    """Reports a criterion that could not be evaluated (verdict BLOCKED)."""
    emit(cid, "BLOCKED", desc, "—", reason)


def finish() -> int:
    """Exit code: 0 when no criterion failed, 1 otherwise."""
    return 0 if _failures == 0 else 1


if __name__ == "__main__":
    sys.exit("check.py is a library; import it from a test case.")
