/*
 *
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file check.h
 * @brief Per-criterion checks for host test cases, reported as rows of the test record.
 *
 * A tc::Criterion collects any number of observations and, when it goes out
 * of scope, prints exactly one result line that tests/common/make_record.py
 * turns into a row of the results table:
 *
 * @code
 * @@<TAB>id<TAB>verdict<TAB>description<TAB>limit<TAB>measured
 * @endcode
 *
 * Unlike assert(), a failing observation does not stop the test: every
 * criterion gets a verdict. Return tc::finish() from main() so the process
 * exit code is non-zero when any criterion failed.
 *
 * @code
 * {
 *     tc::Criterion c("L0-BST-02.a", "Leg 1 alone up to its current limit");
 *     c.near("ILow1 ref at 2 A", boost.getILow1Ref(), 2.0, 1e-5);
 *     c.check("leg 2 disabled", !boost.isLeg2Enabled());
 * }   // result line printed here
 *
 * tc::info("L0-GFL-02.c", "PLL lock time", 0.034, "s");
 * return tc::finish();
 * @endcode
 *
 * Verdicts are defined in docs/testing/test-procedure.md §5.
 */
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

/** @brief Host test-case reporting helpers. */
namespace tc {

/**
 * @brief Number of criteria that failed so far in this process.
 * @return Reference to the process-wide counter.
 */
inline int &failures() {
    static int count = 0;
    return count;
}

/**
 * @brief Makes text safe for one cell of a tab-separated line and a Markdown table.
 * @param text Free text.
 * @return The text with tabs, newlines and pipes replaced by spaces.
 */
inline std::string clean(std::string text) {
    for (char &ch : text) {
        if (ch == '\t' || ch == '\n' || ch == '|') ch = ' ';
    }
    return text;
}

/**
 * @brief Formats a number with six significant digits.
 * @param value Value to format.
 * @return The formatted value.
 */
inline std::string num(double value) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.6g", value);
    return buf;
}

/**
 * @brief Prints one result line on stdout.
 * @param id Criterion ID, e.g. "L0-BST-02.a".
 * @param verdict PASS, FAIL, INFO or BLOCKED.
 * @param desc What the criterion checks.
 * @param limit The pass limit, or "—" when none applies.
 * @param measured What was measured.
 */
inline void emit(const std::string &id, const char *verdict, const std::string &desc,
                 const std::string &limit, const std::string &measured) {
    std::printf("@@\t%s\t%s\t%s\t%s\t%s\n", clean(id).c_str(), verdict,
                clean(desc).c_str(), clean(limit).c_str(), clean(measured).c_str());
    std::fflush(stdout);
}

/**
 * @brief One pass/fail criterion made of any number of observations.
 *
 * The criterion passes when at least one observation was made and none
 * failed. Its result line is printed by end(), called at the latest by the
 * destructor. The record shows the first failing observation and, for
 * near(), the worst error seen.
 */
class Criterion {
public:
    /**
     * @brief Starts a criterion.
     * @param id Criterion ID as listed in the test procedure, e.g. "L0-BST-02.a".
     * @param desc One-line description of what is checked.
     */
    Criterion(std::string id, std::string desc) : _id(std::move(id)), _desc(std::move(desc)) {}
    Criterion(const Criterion &) = delete;
    Criterion &operator=(const Criterion &) = delete;
    /** @brief Prints the result line if end() was not called. */
    ~Criterion() { end(); }

    /**
     * @brief Records a boolean observation.
     * @param what Name of the observation, shown in the record if it is the first to fail.
     * @param ok Observed result.
     * @return @p ok.
     */
    bool check(const char *what, bool ok) {
        ++_observations;
        if (!ok) fail(what);
        return ok;
    }

    /**
     * @brief Records |measured - expected| <= tol.
     * @param what Name of the observation.
     * @param measured Measured value.
     * @param expected Expected value.
     * @param tol Absolute tolerance; the smallest one used becomes the reported limit.
     * @return true when within tolerance (NaN fails).
     */
    bool near(const char *what, double measured, double expected, double tol) {
        const double err = std::fabs(measured - expected);
        ++_observations;
        _tol = _tol < 0.0 ? tol : std::fmin(_tol, tol);
        if (!(err <= tol)) {  // also catches NaN
            fail(what, " = " + num(measured) + ", expected " + num(expected) + " ± " + num(tol));
            return false;
        }
        if (err > _worst_error) _worst_error = err;
        return true;
    }

    /**
     * @brief Records lo <= measured <= hi.
     * @param what Name of the observation.
     * @param measured Measured value.
     * @param lo Lower bound, inclusive.
     * @param hi Upper bound, inclusive.
     * @return true when inside the bounds (NaN fails).
     */
    bool range(const char *what, double measured, double lo, double hi) {
        ++_observations;
        if (!(measured >= lo && measured <= hi)) {
            fail(what, " = " + num(measured) + ", outside [" + num(lo) + ", " + num(hi) + "]");
            return false;
        }
        return true;
    }

    /**
     * @brief Prints the result line now instead of at end of scope.
     * @note Later observations are still counted but no longer reported.
     */
    void end() {
        if (_done) return;
        _done = true;
        const bool ok = _failed == 0 && _observations > 0;
        std::string limit = "all observations hold";
        if (_tol >= 0.0) limit = "abs error ≤ " + num(_tol);
        std::string measured;
        if (_observations == 0) {
            measured = "no observation made";
        } else if (ok) {
            measured = std::to_string(_observations) + "/" + std::to_string(_observations) + " ok";
            if (_tol >= 0.0) measured += ", max abs error " + num(_worst_error);
        } else {
            measured = std::to_string(_failed) + "/" + std::to_string(_observations) +
                       " failed; first: " + _first_failure;
        }
        if (!ok) ++failures();
        emit(_id, ok ? "PASS" : "FAIL", _desc, limit, measured);
    }

private:
    /** @brief Counts a failed observation and keeps the first one for the record. */
    void fail(const char *what, const std::string &detail = "") {
        if (_failed++ == 0) _first_failure = std::string(what ? what : "?") + detail;
    }

    std::string _id;             ///< Criterion ID.
    std::string _desc;           ///< One-line description.
    std::string _first_failure;  ///< First failing observation, with its values.
    int _observations = 0;       ///< Observations made.
    int _failed = 0;             ///< Observations that failed.
    double _worst_error = 0.0;   ///< Largest passing near() error.
    double _tol = -1.0;          ///< Smallest near() tolerance, or -1 if near() was not used.
    bool _done = false;          ///< Result line already printed.
};

/**
 * @brief Reports a measured value that has no stated limit (verdict INFO, never fails).
 * @param id Criterion ID.
 * @param desc What was measured.
 * @param value Measured value.
 * @param unit Unit, or "" for none.
 */
inline void info(const std::string &id, const std::string &desc, double value,
                 const std::string &unit) {
    emit(id, "INFO", desc, "—", num(value) + (unit.empty() ? "" : " " + unit));
}

/**
 * @brief Reports a criterion that could not be evaluated (verdict BLOCKED).
 * @param id Criterion ID.
 * @param desc What the criterion would check.
 * @param reason Why it could not run, e.g. "plant model missing".
 */
inline void blocked(const std::string &id, const std::string &desc, const std::string &reason) {
    emit(id, "BLOCKED", desc, "—", reason);
}

/**
 * @brief Exit code for main().
 * @return 0 when no criterion failed, 1 otherwise.
 */
inline int finish() { return failures() == 0 ? 0 : 1; }

}  // namespace tc
