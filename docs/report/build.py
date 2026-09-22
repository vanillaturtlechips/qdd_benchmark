#!/usr/bin/env python3
"""report.md → (include 해석) → pandoc HTML → weasyprint PDF.   .venv/bin/python docs/report/build.py [--no-figs]
include 문법: {{include:path}}  옵션 |strip-title (첫 H1 제거) |shift=N (헤딩 N단계 내림)"""
import re, subprocess, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[1]; PY = sys.executable

if "--no-figs" not in sys.argv:
    subprocess.run([PY, str(ROOT / "isaac/bench_metrics.py"), "--no-plots"], check=True, cwd=ROOT, capture_output=True)
    subprocess.run([PY, str(ROOT / "isaac/compare_specs.py")], check=True, cwd=ROOT, capture_output=True)
    subprocess.run([PY, str(HERE / "make_tables.py")], check=True, cwd=ROOT, capture_output=True)
    subprocess.run([PY, str(HERE / "make_figs.py")], check=True, cwd=ROOT, capture_output=True)


def include(m):
    parts = m.group(1).split("|"); path = (HERE / parts[0]).resolve(); txt = path.read_text()
    opts = parts[1:]
    if "strip-title" in opts:
        lines = txt.splitlines()
        if lines and lines[0].startswith("# "): lines = lines[1:]
        txt = "\n".join(lines)
    for o in opts:
        if o.startswith("shift="):
            n = int(o.split("=")[1]); txt = re.sub(r"^(#+) ", lambda mm: "#" * (len(mm.group(1)) + n) + " ", txt, flags=re.M)
    return txt


src = (HERE / "report.md").read_text(); full = re.sub(r"\{\{include:([^}]+)\}\}", include, src)
(HERE / "_report_full.md").write_text(full)
subprocess.run(["pandoc", str(HERE / "_report_full.md"), "-s", "--toc", "--toc-depth=2", "-f", "markdown+fenced_divs", "-t", "html5", "--metadata", "lang=ko",
                "--css", "report.css", "-o", str(HERE / "report.html")], check=True, cwd=HERE, capture_output=True)
html = (HERE / "report.html").read_text()
# 실측 DM 행 강조: 첫 셀이 DM/dm 으로 시작하는 <tr> 에 class="dm"
def mark(m):
    row = m.group(0); first = re.search(r"<td[^>]*>(.*?)</td>", row, flags=re.S)
    txt = re.sub(r"<[^>]+>", "", first.group(1)).strip() if first else ""
    if not (txt.lower().startswith("dm") or txt.startswith("Damiao")): return row
    return re.sub(r'^<tr( class=")?', lambda mm: '<tr class="dm ' if mm.group(1) else '<tr class="dm"', row, count=1)
html = re.sub(r"<tr[^>]*>.*?</tr>", mark, html, flags=re.S)
(HERE / "report.html").write_text(html)
import weasyprint  # noqa: E402
weasyprint.HTML(str(HERE / "report.html"), base_url=str(HERE)).write_pdf(str(HERE / "report.pdf"))
n = subprocess.run(["pdfinfo", str(HERE / "report.pdf")], capture_output=True, text=True).stdout
print("→ docs/report/report.pdf", [l for l in n.splitlines() if l.startswith("Pages")])
