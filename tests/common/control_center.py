#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file control_center.py
## @brief Writes tests/out/control-center.html: where the validation stands, one tab per stage.
## @par Usage
##   python3 tests/common/control_center.py [--out tests/out]
##   Also run at the end of every campaign (tests/run_all.sh) and study (tests/l1/study.py).
## @par What it shows
##   Tabs from tests/catalogue.json (Test bench, Model, Boost, Grid observer, Inverter, Application).
##   For each test: definition, implementation state, and the latest result found in tests/out
##   (newest campaign containing that test), with a link to that campaign's review page; study
##   runs listed under the tests they inform (exploratory, they never set a verdict).
## @return 0; a broken catalogue is fatal (1).
"""Reads the catalogue and the *.log result lines of every campaign under tests/out; changes nothing."""
from pathlib import Path
import argparse,datetime as dt,fnmatch,html,json,os,re,subprocess,sys

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
sys.path.insert(0,str(HERE))
from make_record import parse,findings_register,finding_for  # noqa: E402
import catalogue  # noqa: E402

e=html.escape


def md(s):
    """@brief Escaped text with `code` spans rendered."""
    return re.sub(r'`([^`]+)`',r'<code>\1</code>',e(s or ''))
# Display state of a test, worst first. Colour is never alone: each has an icon and a word.
STATES=[('fail','✗','Failing'),('known','✗','Failing (known)'),('blocked_run','–','Blocked in last run'),
        ('pass','✓','Passed'),('info','i','Run (INFO only)'),('not_run','○','Not run yet'),
        ('blocked','–','Blocked'),('planned','◌','Not implemented')]
LABEL={k:(i,w) for k,i,w in STATES}


def campaigns(out):
    """@brief Every campaign folder with result lines. @return list of dicts, newest first."""
    found=[]
    for d in out.iterdir():
        if not d.is_dir() or d.name.startswith('study_'):continue
        logs=sorted(d.glob('*.log'))
        rows=parse(logs) if logs else []
        if not rows:continue
        when=max(l.stat().st_mtime for l in logs)
        m=re.match(r'\d{4}-\d\d-\d\d_([0-9a-f]{7,})(-dirty)?',d.name)
        found.append({'dir':d,'rows':rows,'when':when,'commit':(m.group(1)+(m.group(2) or '')) if m else '',
                      'report':(d/'report.html').exists()})
    return sorted(found,key=lambda c:-c['when'])


def studies(out):
    """@brief Every study folder: name, cases, report. Newest first."""
    found=[]
    for s in out.glob('study_*/status.json'):
        st=json.loads(s.read_text())
        found.append({'dir':s.parent,'name':st['study'].get('name',s.parent.name),'title':st['study'].get('title',''),
                      'started':st.get('started',''),'finished':st.get('finished'),'cases':st['cases']})
    return sorted(found,key=lambda s:s['started'],reverse=True)


def latest_results(camps):
    """@brief {case ID: (campaign, [rows])} from the newest campaign that ran each case."""
    latest={}
    for c in camps:  # newest first
        by={}
        for r in c['rows']:by.setdefault(r['id'].split('.')[0],[]).append(r)
        for cid,rows in by.items():latest.setdefault(cid,(c,rows))
    return latest


def newest_of_level(camps,level):
    """@brief Newest campaign that ran any criterion of a level (L0/L1), or None."""
    return next((c for c in camps if any(r['id'][:2]==level for r in c['rows'])),None)


def state_of(test,result,findings):
    """@brief Display state of one test from its implementation state and latest result."""
    if result is None:
        return {'planned':'planned','blocked':'blocked'}.get(test['status'],'not_run')
    rows=result[1]
    fails=[r for r in rows if r['verdict']=='FAIL']
    if fails:return 'known' if all(finding_for(r['id'],findings) for r in fails) else 'fail'
    if rows and all(r['verdict']=='BLOCKED' for r in rows):return 'blocked_run'
    if any(r['verdict']=='PASS' for r in rows):return 'pass'
    return 'info'


def ago(ts):
    d=dt.datetime.now()-dt.datetime.fromtimestamp(ts)
    if d.days:return f'{d.days} d ago'
    h=d.seconds//3600
    return f'{h} h ago' if h else f'{d.seconds//60} min ago'


def git(*a):
    try:return subprocess.run(['git',*a],cwd=REPO,capture_output=True,text=True,timeout=10).stdout.strip()
    except Exception:return ''


def bar(counts,total):
    """@brief Segmented progress bar plus its text (the text carries the meaning, the bar the shape)."""
    segs=''.join(f'<span class="seg s-{k}" style="flex:{counts[k]}" title="{e(LABEL[k][1])}: {counts[k]}"></span>'
                 for k,_,_ in STATES if counts.get(k))
    return f'<div class="bar" role="img" aria-label="progress">{segs}</div>'


def summary_text(counts):
    return ' · '.join(f'{LABEL[k][0]} {counts[k]} {LABEL[k][1].lower()}' for k,_,_ in STATES if counts.get(k))


def badge(k):
    i,w=LABEL[k]
    return f'<span class="badge b-{k}">{i} {e(w)}</span>'


def study_links(test,studs):
    links=[]
    for s in studs:
        for c in s['cases']:
            if any(fnmatch.fnmatch(c['folder'],g) for g in test.get('studies',[])):
                links.append(f'<a href="{e(s["dir"].name)}/report.html#{e(c["folder"])}">{e(s["name"])} / {e(c["folder"])}</a> ({e(c["state"])})')
    return links


def test_row(t,result,state,studs,findings,hardware):
    rel=lambda c:f'{e(c["dir"].name)}/report.html' if c['report'] else f'{e(c["dir"].name)}/record.md'
    when=''
    if result:
        c=result[0]
        when=f'<a href="{rel(c)}">{dt.datetime.fromtimestamp(c["when"]).strftime("%Y-%m-%d %H:%M")}</a><div class="sub">{ago(c["when"])} · {e(c["commit"])}</div>'
    issues=sorted({f['issue'] for r in (result[1] if result else []) if r['verdict']=='FAIL'
                   for f in [finding_for(r['id'],findings)] if f})
    det=[]
    if t.get('criterion'):det.append(f'<p><b>Criterion.</b> {md(t["criterion"])}</p>')
    if t.get('conditions'):det.append(f'<p><b>Conditions.</b> {md(t["conditions"])}</p>')
    if t.get('harness'):det.append(f'<p><b>Harness.</b> {md(t["harness"])}</p>')
    if t.get('source'):det.append(f'<p><b>Source.</b> <code>{e(t["source"])}</code></p>')
    det.append(f'<p><b>Implementation.</b> {e(catalogue.STATUS_TEXT[t["status"]])}'+(f' — {md(t["status_note"])}' if t.get('status_note') else '')+'</p>')
    if result:
        det.append('<table class="crit"><tr><th>Criterion</th><th>Verdict</th><th>Limit</th><th>Measured</th></tr>'+''.join(
            f'<tr><td>{e(r["id"])}</td><td>{e(r["verdict"])}</td><td>{e(r["limit"])}</td><td>{e(r["measured"])}</td></tr>'
            for r in sorted(result[1],key=lambda r:r['id']))+'</table>')
    sl=study_links(t,studs)
    if sl:det.append('<p><b>Study runs (exploratory, no verdict).</b><br>'+'<br>'.join(sl)+'</p>')
    hw=f'<td>{e(t.get("hardware","—"))}</td>' if hardware else ''
    return (f'<tr class="r-{state}"><td class="id">{e(t["id"])}</td>'
            f'<td><details><summary>{md(t["title"])}</summary>{"".join(det)}</details></td>'
            f'<td>{badge(state)}{"<div class=sub>partly implemented</div>" if t["status"]=="partial" else ""}{"".join(f" <span class=sub>{e(i)}</span>" for i in issues)}</td><td>{when}</td>{hw}</tr>')


def page(cat,camps,studs,out):
    findings=findings_register()
    latest=latest_results(camps)
    for t in cat['tests']:  # a check that only reports failures passed if the newest campaign of its level is silent
        if t.get('reports_only_failures'):
            c=newest_of_level(camps,t['level'])
            if c and t['id'] not in {r['id'].split('.')[0] for r in c['rows']}:
                latest[t['id']]=(c,[{'id':t['id'],'verdict':'PASS','desc':'','limit':'','measured':'not reported: the campaign ran','log':''}])
    states={t['id']:state_of(t,latest.get(t['id']),findings) for t in cat['tests']}
    known_ids=set(cat['by_id'])|set(cat.get('retired',{}))
    # Results with no catalogue entry, from the newest campaign of each level only (older ones may predate a rename).
    newest_by_level={lv:c for lv in ('L0','L1') for c in [newest_of_level(camps,lv)] if c}
    stray=sorted({(r['id'].split('.')[0],c['dir'].name) for lv,c in newest_by_level.items() for r in c['rows']
                  if r['id'][:2]==lv and r['id'].split('.')[0] not in known_ids})
    head=git('log','-1','--format=%h %s')
    now=dt.datetime.now().strftime('%Y-%m-%d %H:%M')
    p=[f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Validation control center</title><style>
:root{{--bg:#fcfcfb;--panel:#ffffff;--ink:#0b0b0b;--ink2:#52514e;--muted:#8a8984;--line:#e4e2dc;
--good:#008300;--bad:#d03b3a;--warn:#b07800;--none:#c9c7c0;--none2:#e9e7e1;--accent:#2a78d6}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#161615;--panel:#1e1e1c;--ink:#f1f0ec;--ink2:#b5b3ad;--muted:#8a8984;--line:#34332f;
--good:#3fb950;--bad:#f0605f;--warn:#e0a521;--none:#5a5953;--none2:#3a3935;--accent:#6aa7ee}}}}
body{{background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,sans-serif;margin:0;padding:24px 16px}}
main{{max-width:1100px;margin:auto}} h1{{font-size:22px;margin:0}} h2{{font-size:18px;margin:28px 0 6px}}
.sub{{color:var(--ink2);font-size:13px}} a{{color:var(--accent)}}
.overview{{display:grid;gap:8px;margin:16px 0}} .ov{{display:grid;grid-template-columns:150px 1fr;gap:4px 16px;align-items:center;
 background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 14px;cursor:pointer;text-align:left;font:inherit;color:inherit}}
.ov:hover{{border-color:var(--accent)}} .ov b{{font-size:15px}} .ov .sub{{grid-column:2}}
.bar{{display:flex;gap:2px;height:12px;border-radius:4px;overflow:hidden;background:var(--none2)}} .seg{{min-width:3px}}
.s-pass,.s-info{{background:var(--good)}} .s-fail,.s-known{{background:var(--bad)}} .s-blocked_run,.s-blocked{{background:var(--warn)}}
.s-not_run{{background:var(--none)}} .s-planned{{background:var(--none2)}}
.tabs{{display:flex;flex-wrap:wrap;gap:4px;border-bottom:2px solid var(--line);margin-top:24px}}
.tabs button{{font:inherit;font-size:14px;background:none;border:0;border-bottom:3px solid transparent;margin-bottom:-2px;padding:6px 12px;cursor:pointer;color:var(--ink2)}}
.tabs button[aria-selected=true]{{color:var(--ink);border-bottom-color:var(--accent);font-weight:600}}
section[hidden]{{display:none}} .q{{margin:12px 0 4px}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin:6px 0}} table.tests{{table-layout:fixed}} table.tests col.c1{{width:96px}} table.tests col.c3{{width:150px}} table.tests col.c4{{width:130px}} table.tests col.c5{{width:110px}} td,th{{border-bottom:1px solid var(--line);padding:6px;text-align:left;vertical-align:top}}
th{{color:var(--ink2);font-weight:600}} td.id{{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font-family:ui-monospace,monospace;font-size:12px}}
details summary{{cursor:pointer}} details p{{margin:6px 0;color:var(--ink2)}} details[open] summary{{font-weight:600}}
table.crit{{margin:6px 0 10px;font-size:12px}} code{{font-size:12px}}
.badge{{display:inline-block;padding:0 8px;border-radius:10px;font-size:12px;font-weight:600;border:1px solid var(--line);white-space:nowrap}}
.b-pass,.b-info{{color:var(--good)}} .b-fail{{background:var(--bad);color:#fff;border-color:var(--bad)}} .b-known{{color:var(--bad);border-color:var(--bad)}}
.b-blocked_run,.b-blocked{{color:var(--warn)}} .b-not_run,.b-planned{{color:var(--ink2)}}
.wrap{{overflow-x:auto}} .latest{{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:8px 14px;margin:12px 0}} .latest ul{{margin:4px 0;padding-left:18px}} .latest li{{margin:2px 0}} .note{{border-left:3px solid var(--line);padding:2px 12px;color:var(--ink2);font-size:13px;margin:8px 0}}
@media (max-width:640px){{.ov{{grid-template-columns:1fr}} .ov .sub{{grid-column:1}}}}
</style></head><body><main>
<h1>Validation control center</h1>
<div class="sub">Generated {e(now)} · repository at {e(head)} · catalogue {e(cat.get("version",""))} (<code>tests/catalogue.json</code>, procedure §6) ·
regenerate: <code>python3 tests/common/control_center.py</code></div>
<p class="q">Where the validation stands. Each tab is one stage: the test bench first, then the model, each object alone, then the application. A test's result is its newest campaign; study runs are linked as exploratory evidence and never set a verdict.</p>
''']
    # Latest runs first: where the newest results are, without opening the Runs tab.
    runs=[(c['when'],'campaign',c['dir'].name,f'{c["dir"].name}/report.html' if c['report'] else f'{c["dir"].name}/record.md',
           ' · '.join(f'{sum(r["verdict"]==v for r in c["rows"])} {v}' for v in ('PASS','FAIL','INFO','BLOCKED'))) for c in camps[:4]]
    for s_ in studs[:2]:
        try:when=dt.datetime.fromisoformat(s_['started']).timestamp()
        except ValueError:when=0
        runs.append((when,'study',s_['name'],f'{s_["dir"].name}/report.html',f'{sum(c["state"]=="done" for c in s_["cases"])}/{len(s_["cases"])} cases done'))
    runs.sort(reverse=True)
    p.append('<div class="latest"><b>Latest runs</b> <span class="sub">(newest first; reload this page after a run)</span><ul>'+''.join(
        f'<li><a href="{e(link)}">{e(name)}</a> <span class="sub">{e(kind)} · {dt.datetime.fromtimestamp(w).strftime("%Y-%m-%d %H:%M") if w else ""} · {ago(w) if w else ""} · {e(summary)}</span></li>'
        for w,kind,name,link,summary in runs[:5])+'</ul></div>')
    p.append('<div class="overview">')
    tabs=cat['tabs']
    for tab in tabs:
        tests=[t for t in cat['tests'] if t['tab']==tab['id']]
        counts={k:sum(states[t['id']]==k for t in tests) for k,_,_ in STATES}
        p.append(f'<button class="ov" data-go="{e(tab["id"])}"><b>{e(tab["title"])}</b>{bar(counts,len(tests))}'
                 f'<span class="sub">{len(tests)} tests: {e(summary_text(counts))}</span></button>')
    p.append('</div><div class="sub">Legend: ✓ passed · ✗ failing (filled: new, outlined: known issue) · – blocked · ○ implemented, not run yet · ◌ not implemented</div>')
    p.append('<nav class="tabs" role="tablist">'+''.join(
        f'<button role="tab" data-tab="{e(t["id"])}" aria-selected="false">{e(t["title"])}</button>' for t in tabs)+
        '<button role="tab" data-tab="runs" aria-selected="false">Runs</button></nav>')
    for tab in tabs:
        tests=[t for t in cat['tests'] if t['tab']==tab['id']]
        counts={k:sum(states[t['id']]==k for t in tests) for k,_,_ in STATES}
        p.append(f'<section id="tab-{e(tab["id"])}" hidden><p class="q"><b>{e(tab["question"])}</b></p>'
                 f'{bar(counts,len(tests))}<div class="sub">{len(tests)} tests: {e(summary_text(counts))}</div>')
        for note in tab.get('notes',[]):
            p.append(f'<div class="note">{e(re.sub(r"\[([^]]+)\]\([^)]+\)",r"\1",note).replace("**","").replace("`",""))}</div>')
        for lv,name in (('L0','L0 · logic, no model'),('L1','L1 · on the switched model'),('L2','L2 · on the board (bench)')):
            rows=[t for t in tests if t['level']==lv]
            if not rows:continue
            hw=tab['id']=='model'
            p.append(f'<h2>{e(name)}</h2><div class="wrap"><table class="tests"><colgroup><col class="c1"><col><col class="c3"><col class="c4">'+('<col class="c5">' if hw else '')+'</colgroup><tr><th>ID</th><th>Test (click for definition and results)</th><th>State</th><th>Last run</th>'
                     +('<th>Hardware</th>' if hw else '')+'</tr>')
            for t in rows:p.append(test_row(t,latest.get(t['id']),states[t['id']],studs,findings,hw))
            p.append('</table></div>')
        p.append('</section>')
    p.append('<section id="tab-runs" hidden><h2>Campaigns</h2><div class="wrap"><table><tr><th>Campaign</th><th>When</th><th>Results</th></tr>')
    for c in camps[:30]:
        v={k:sum(r['verdict']==k for r in c['rows']) for k in ('PASS','FAIL','INFO','BLOCKED')}
        link=f'{e(c["dir"].name)}/report.html' if c['report'] else f'{e(c["dir"].name)}/record.md'
        p.append(f'<tr><td><a href="{link}">{e(c["dir"].name)}</a></td><td>{dt.datetime.fromtimestamp(c["when"]).strftime("%Y-%m-%d %H:%M")}</td>'
                 f'<td>{v["PASS"]} PASS · {v["FAIL"]} FAIL · {v["INFO"]} INFO · {v["BLOCKED"]} BLOCKED</td></tr>')
    p.append('</table></div><h2>Studies (exploratory)</h2><div class="wrap"><table><tr><th>Study</th><th>Started</th><th>Cases</th></tr>')
    for s in studs:
        done=sum(c['state']=='done' for c in s['cases'])
        p.append(f'<tr><td><a href="{e(s["dir"].name)}/report.html">{e(s["name"])}</a><div class="sub">{e(s["title"])}</div></td>'
                 f'<td>{e(s["started"])}</td><td>{done}/{len(s["cases"])} done{"" if s["finished"] else " (running)"}</td></tr>')
    p.append('</table></div>')
    if stray:
        p.append('<h2>Results with no catalogue entry</h2><p class="sub">From the newest campaign of each level. Add them to <code>tests/catalogue.json</code> or rename the check. '
                 'A bare object code (e.g. L0-BST) means that test program did not complete.</p><ul>'+
                 ''.join(f'<li><code>{e(i)}</code> in {e(d)}</li>' for i,d in stray)+'</ul>')
    p.append('</section>')
    p.append('''<script>
const tabs=[...document.querySelectorAll('[data-tab]')];
function show(id){if(!document.getElementById('tab-'+id))id=tabs[0].dataset.tab;
 tabs.forEach(b=>b.setAttribute('aria-selected',b.dataset.tab===id));
 document.querySelectorAll('section[id^=tab-]').forEach(s=>s.hidden=s.id!=='tab-'+id);
 try{history.replaceState(null,'','#'+id)}catch(_){}}
tabs.forEach(b=>b.onclick=()=>show(b.dataset.tab));
document.querySelectorAll('[data-go]').forEach(b=>b.onclick=()=>{show(b.dataset.go);document.querySelector('.tabs').scrollIntoView()});
show(location.hash.slice(1));
</script></main></body></html>''')
    return '\n'.join(p)


def main():
    a=argparse.ArgumentParser();a.add_argument('--out',type=Path,default=REPO/'tests/out')
    args=a.parse_args()
    try:cat=catalogue.load()
    except ValueError as ex:print(ex,file=sys.stderr);return 1
    args.out.mkdir(parents=True,exist_ok=True)
    target=args.out/'control-center.html'
    tmp=target.with_suffix('.tmp');tmp.write_text(page(cat,campaigns(args.out),studies(args.out),args.out),encoding='utf-8')
    os.replace(tmp,target)
    print(target)
    return 0


if __name__=='__main__':sys.exit(main())
