#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file recap.py
## @brief Recap figures of the critical-task optimizations from their L0-TIM-03 results.
##
## Reads recap.json (one line per row of the changes table in docs/testing/timing-optimization.md, with the
## campaign that measured it) and writes two standalone SVGs with the review page's L0-TIM-03 drawing (same
## sections, colours and scale):
##   timing-recap-summary.svg  one bar per line: the interrupt's mean work per period, averaged over the stages
##   timing-recap-stages.svg   per stage, one bar per line
## Every bar sums to the measured mean (task API + task tick); the black mark is the worst case.
##
## @par Usage
##   python3 tests/l0/timing/recap.py [--out docs/testing/images]
import argparse, html, json, re, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / 'tests/common'))
import review_report as rr  # noqa: E402


def load(campaign):
    return json.load(open(REPO / 'tests/out' / campaign / 'l0_tim01/timing_report.json'))


def legend_groups(bar_sets):
    """@brief Union of the groups drawn, in drawing order: a section shown coarse in an old run (writes as one
    block) is listed next to the split sections of the newer runs."""
    out = []
    for bars in bar_sets:
        for b in bars:
            for gname, hue, k, n, key, text, *_ in b['segs']:
                g = next((x for x in out if x[0] == gname), None)
                if g is None:
                    g = (gname, hue, [])
                    out.append(g)
                if (key, text) not in g[2]:
                    g[2].append((key, text))
    return out


def summary_bars(lines):
    """@brief One bar per line: each section's mean averaged over the stages; worst = the worst stage."""
    bars = []
    for ln in lines:
        per_stage = rr.bar_rows(load(ln['campaign']), 'tim03')
        n = len(per_stage)
        acc = {}
        for b in per_stage:
            for seg in b['segs']:
                key = seg[4]
                if key not in acc:
                    acc[key] = list(seg)
                    acc[key][6] = 0.0
                acc[key][6] += seg[6] / n
        bars.append({'bucket': ln['text'], 'tag': '', 'line': ln['tag'], 'segs': [tuple(v) for v in acc.values()],
                     'worst': max(b['worst'] for b in per_stage), 'mean': sum(b['mean'] for b in per_stage) / n})
    return add_gains(bars, lines, lambda b: b['line'])


def stage_bars(lines):
    """@brief Per stage, one bar per line, in the order of the lines."""
    sets = [rr.bar_rows(load(ln['campaign']), 'tim03', ln['tag']) for ln in lines]
    stages = [b['bucket'] for b in sets[0]]
    bars = [next(b for b in s if b['bucket'] == st) for st in stages for s in sets]
    return add_gains(bars, lines, lambda b: b['tag'])


def add_gains(bars, lines, line_of):
    """@brief Reference means of each bar: the first line ('total') and the line named by 'step', same stage."""
    step_of = {ln['tag']: ln.get('step') for ln in lines}
    first = lines[0]['tag']
    mean_at = {(b['bucket'], line_of(b)): b['mean'] for b in bars}
    for b in bars:
        key = b['bucket'] if b['tag'] else None  # summary bars: one per line, matched by line only
        look = (lambda t: mean_at.get((b['bucket'], t))) if key else \
               (lambda t: next((x['mean'] for x in bars if line_of(x) == t), None))
        b['total'] = look(first) if line_of(b) != first else None
        s = step_of.get(line_of(b))
        b['step'] = look(s) if s and s != first else None
    return bars


def side_text(b):
    """@brief Right column: mean, total gain (against the first line), step gain (against the change's own base)."""
    parts = [f'mean {b["mean"]:.1f}']
    if b.get('total') is not None:
        parts.append(f'total {b["mean"] - b["total"]:+.1f}')
    if b.get('step') is not None:
        parts.append(f'step {b["mean"] - b["step"]:+.1f}')
    parts.append(f'worst {b["worst"]:.1f}')
    return ' · '.join(parts)


def figure(bars, groups, title, label_w, W=1240, period_us=100.0, budget_us=80.0):
    """@brief Standalone SVG: title, legend, bars (fitted), names on the left, mean and gains in a right column."""
    e = html.escape
    worst = max(b['worst'] for b in bars)
    xmax = max(period_us, 10 * (int((worst + 6) / 10) + 1))
    ys, bar, H = rr._layout(bars)
    right = 330
    x = lambda v: label_w + 8 + (W - label_w - 8 - right) * min(v, xmax) / xmax

    body = rr._bars_svg(bars, x, ys, bar, H, W, xmax, budget_us, period_us, 'overview', lambda b: '')
    col = ''.join(f'<text x="{x(xmax) + 14:.1f}" y="{y + bar / 2 + 4}" fill="{rr.INK}">{e(side_text(b))}</text>'
                  for b, y in zip(bars, ys))
    labels = rr._labels_svg(bars, ys, bar, H, label_w, False)
    labels = labels[labels.index('>') + 1:-len('</svg>')]
    body_inner = body[body.index('>') + 1:-len('</svg>')]
    leg = rr.legend_svg(groups, W)
    lh = float(re.search(r'viewBox="0 0 \S+ (\S+)"', leg).group(1))
    leg_inner = leg[leg.index('>') + 1:-len('</svg>')]
    top = 28
    total_h = top + lh + 8 + H
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{total_h:.0f}" viewBox="0 0 {W} {total_h:.0f}" '
            f'style="font:12px system-ui,sans-serif;background:{rr.SURFACE}">'
            f'<rect width="{W}" height="{total_h:.0f}" fill="{rr.SURFACE}"/>'
            f'<text x="0" y="18" fill="{rr.INK}" font-size="15" font-weight="600">{e(title)}</text>'
            f'<g transform="translate(0,{top})">{leg_inner}</g>'
            f'<g transform="translate(0,{top + lh + 8:.0f})">{body_inner}{labels}{col}</g></svg>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(REPO / 'docs/testing/images'))
    a = ap.parse_args()
    lines = json.load(open(HERE / 'recap.json'))['lines']
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    summ = summary_bars(lines)
    (out / 'timing-recap-summary.svg').write_text(figure(
        summ, legend_groups([summ]),
        'L0-TIM-03 · interrupt work per 100 µs period, mean over the 9 stages (µs)', label_w=230))

    st = stage_bars(lines)
    (out / 'timing-recap-stages.svg').write_text(figure(
        st, legend_groups([st]), 'L0-TIM-03 per stage · one bar per line of the changes table (µs)', label_w=190))
    for f in ('timing-recap-summary.svg', 'timing-recap-stages.svg'):
        print(out / f)


if __name__ == '__main__':
    main()
