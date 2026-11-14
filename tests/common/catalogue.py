#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file catalogue.py
## @brief The validation catalogue (tests/catalogue.json): load, check, and write procedure §6 from it.
## @par Usage
##   python3 tests/common/catalogue.py              # check the catalogue, print a summary
##   python3 tests/common/catalogue.py --procedure  # rewrite §6 of docs/testing/test-procedure.md
## @par Catalogue
##   tabs: one per validation stage (Test bench, Model, Boost, Grid observer, Inverter, Application).
##   tests: id, tab, level (L0/L1/L2/D), title, criterion, conditions, source, status, status_note, and
##   optional harness, verdict_type, studies (study case folders, glob), hardware (Model tab).
##   status is the implementation state (implemented | partial | planned | blocked); verdicts come
##   only from campaign results (tests/common/control_center.py).
## @return 1 if the catalogue is inconsistent.
"""One definition of every test; the procedure and the control center are generated from it."""
from pathlib import Path
import argparse,json,re,sys

REPO=Path(__file__).resolve().parents[2]
CATALOGUE=REPO/'tests/catalogue.json'
PROCEDURE=REPO/'docs/testing/test-procedure.md'
STATUSES=('implemented','partial','planned','blocked')
STATUS_TEXT={'implemented':'Implemented','partial':'Partly implemented','planned':'Planned','blocked':'Blocked'}
ID=re.compile(r'^(L[012]|D)-[A-Z]{3}-\d\d$')  # L2: on the board (bench); D: averaged-model design tests (ADR 0004)


def load():
    """@brief The catalogue, checked. @return dict with 'tabs', 'tests', 'by_id'. Raises ValueError."""
    cat=json.loads(CATALOGUE.read_text(encoding='utf-8'))
    tabs={t['id'] for t in cat['tabs']}
    errors=[];seen=set()
    for t in cat['tests']:
        i=t.get('id','?')
        if not ID.match(i):errors.append(f'{i}: ID must look like L1-BST-01')
        if i in seen:errors.append(f'{i}: duplicate')
        seen.add(i)
        if t.get('tab') not in tabs:errors.append(f'{i}: unknown tab {t.get("tab")}')
        if t.get('status') not in STATUSES:errors.append(f'{i}: status must be one of {STATUSES}')
        if t.get('level')!=i.split('-')[0]:errors.append(f'{i}: level does not match the ID')
        if not t.get('title'):errors.append(f'{i}: no title')
    if errors:raise ValueError('\n'.join(errors))
    cat['by_id']={t['id']:t for t in cat['tests']}
    return cat


def cell(s):
    return str(s or '').replace('|','\\|').replace('\n',' ')


def procedure_section(cat):
    """@brief Markdown of §6, generated (never edit it by hand)."""
    out=['## 6. Test catalogue','',
         '<!-- Generated from tests/catalogue.json by tests/common/catalogue.py --procedure. Edit the catalogue, not this section. -->','',
         'Every test is defined once, in [`tests/catalogue.json`](../../tests/catalogue.json), grouped by validation stage. '
         'The [control center](#85-reviewing-a-campaign) shows the same tests with their latest results. '
         '"Status" is the implementation state; verdicts come from campaigns (§8). '
         'The "Source" names the symbol that states the behaviour or the limit (names survive refactoring, line numbers do not).','']
    for n,tab in enumerate(cat['tabs'],1):
        tests=[t for t in cat['tests'] if t['tab']==tab['id']]
        out+=[f'### 6.{n} {tab["title"]}','',tab['question'],'']
        for note in tab.get('notes',[]):out+=[note,'']
        out+=['| ID | Test | Criterion and conditions | Source | Status |','|---|---|---|---|---|']
        for t in tests:
            crit=t.get('criterion') or ''
            if t.get('conditions'):crit+=(' ' if crit else '')+f'*Conditions:* {t["conditions"]}'
            if t.get('harness'):crit+=f' *Harness:* {t["harness"]}'
            if t.get('verdict_type'):crit+=f' *Verdict:* {t["verdict_type"]}'
            status=STATUS_TEXT[t['status']]
            if t.get('status_note') and t['status_note'].lower()!=status.lower():status+=f': {t["status_note"]}'
            out.append(f'| {t["id"]} | {cell(t["title"])} | {cell(crit)} | {cell(t.get("source"))} | {cell(status)} |')
        out.append('')
    return '\n'.join(out)


def write_procedure(cat):
    if not PROCEDURE.exists():
        raise SystemExit(f'no {PROCEDURE.relative_to(REPO)} on this branch: nothing to regenerate')
    text=PROCEDURE.read_text(encoding='utf-8')
    a=text.index('## 6. Test catalogue');b=text.index('## 7. Running a campaign')
    PROCEDURE.write_text(text[:a]+procedure_section(cat)+'\n'+text[b:],encoding='utf-8')


def main():
    p=argparse.ArgumentParser();p.add_argument('--procedure',action='store_true',help='rewrite procedure §6')
    args=p.parse_args()
    try:cat=load()
    except ValueError as e:print(e,file=sys.stderr);return 1
    for tab in cat['tabs']:
        tests=[t for t in cat['tests'] if t['tab']==tab['id']]
        print(f'{tab["title"]:14} {len(tests):3} tests  '+'  '.join(f'{s} {sum(t["status"]==s for t in tests)}' for s in STATUSES))
    if args.procedure:write_procedure(cat);print('wrote',PROCEDURE.relative_to(REPO),'§6')
    return 0


if __name__=='__main__':sys.exit(main())
