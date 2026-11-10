#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file design_report.py
## @brief Render a design-test report (D level, ADR 0004) from report.md to report.html, styled like the
##        campaign review pages, with the Mermaid diagram drawn.
## @par Usage
##   python3 tests/common/design_report.py docs/testing/design/D-GFL-01_IEC/report.md [...]
## @par Supported Markdown (what the design scripts write)
##   headings #/##/###, paragraphs, pipe tables, images, ```mermaid blocks, numbered and bullet lists,
##   links (links to another report.md point to its report.html), **bold**, `code`.
"""Presentation only: the Markdown file stays the record."""
import html, re, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from review_report import page_head  # noqa: E402

MERMAID = 'https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js'


def inline(s):
    s = html.escape(s, quote=False)
    s = re.sub(r'`([^`]+)`', r'<code>\1</code>', s)
    s = re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', s)
    s = re.sub(r'!\[([^\]]*)\]\(([^)]+)\)', r'<img alt="\1" src="\2">', s)
    def link(m):
        url = re.sub(r'report\.md(#.*)?$', lambda q: 'report.html' + (q.group(1) or ''), m.group(2))
        return f'<a href="{url}">{m.group(1)}</a>'
    return re.sub(r'(?<!!)\[([^\]]+)\]\(([^)]+)\)', link, s)


def cells(line):
    return [c.strip().replace('\\|', '|') for c in re.split(r'(?<!\\)\|', line.strip().strip('|'))]


def render(md_text, title):
    out, lines, i = [], md_text.splitlines(), 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith('```mermaid'):
            j = i + 1; block = []
            while j < len(lines) and not lines[j].startswith('```'): block.append(lines[j]); j += 1
            out.append('<div class="wrap"><pre class="mermaid">' + html.escape('\n'.join(block)) + '</pre></div>'); i = j + 1; continue
        m = re.match(r'^(#{1,3}) (.*)', ln)
        if m:
            n = len(m.group(1)); out.append(f'<h{n}>{inline(m.group(2))}</h{n}>'); i += 1; continue
        if ln.startswith('|'):
            rows = []
            while i < len(lines) and lines[i].startswith('|'): rows.append(lines[i]); i += 1
            head, body = cells(rows[0]), [cells(r) for r in rows[2:]]
            out.append('<div class="wrap"><table><tr>' + ''.join(f'<th>{inline(c)}</th>' for c in head) + '</tr>'
                       + ''.join('<tr>' + ''.join(f'<td>{inline(c)}</td>' for c in r) + '</tr>' for r in body) + '</table></div>')
            continue
        m = re.match(r'^!\[([^\]]*)\]\(([^)]+)\)\s*$', ln)
        if m:
            out.append(f'<figure><img alt="{html.escape(m.group(1))}" src="{html.escape(m.group(2))}"><figcaption>{html.escape(m.group(1))}</figcaption></figure>'); i += 1; continue
        if re.match(r'^(\d+\.|-) ', ln):
            tag = 'ol' if ln[0].isdigit() else 'ul'; items = []
            while i < len(lines) and re.match(r'^(\d+\.|-) ', lines[i]):
                item = re.sub(r'^(\d+\.|-) ', '', lines[i]); i += 1
                while i < len(lines) and lines[i].startswith('   ') and lines[i].strip(): item += ' ' + lines[i].strip(); i += 1
                items.append(f'<li>{inline(item)}</li>')
            out.append(f'<{tag}>' + ''.join(items) + f'</{tag}>'); continue
        if not ln.strip(): i += 1; continue
        para = [ln]; i += 1
        while i < len(lines) and lines[i].strip() and not re.match(r'^(#|\||```|!\[|\d+\. |- )', lines[i]): para.append(lines[i]); i += 1
        out.append('<p>' + inline(' '.join(para)) + '</p>')
    return (page_head(html.escape(title)) + '\n'.join(out)
            + f'\n<script src="{MERMAID}"></script><script>mermaid.initialize({{startOnLoad:true,theme:"neutral"}});</script></main></body></html>\n')


def write(md_path):
    md_path = Path(md_path); text = md_path.read_text(encoding='utf-8')
    title = next((l[2:] for l in text.splitlines() if l.startswith('# ')), md_path.parent.name)
    target = md_path.with_suffix('.html'); target.write_text(render(text, title), encoding='utf-8'); return target


if __name__ == '__main__':
    for p in sys.argv[1:]: print(write(p))
