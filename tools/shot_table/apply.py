#!/usr/bin/env python3
"""Shot table: Excel -> src/Constants/ShotTableData.h -> (optionally) the Mega.

  python3 tools/shot_table/apply.py            check the Excel, show what changes, write the header
  python3 tools/shot_table/apply.py --upload   ...then build and upload the Mega (pio)
  python3 tools/shot_table/apply.py --check    check only, write nothing
  python3 tools/shot_table/apply.py --export   rebuild shot_table.xlsx from the current header

The Excel (tools/shot_table/shot_table.xlsx), sheet "射擊表", row 1 = headers,
one measured point per row:

  洞口      Dashboard key 1-9 (1 left-bottom, 2 left-mid, 3 left-top ... 9 right-top)
  距離(m)   the camera distance shown on the Dashboard while locked, 0.5-15 m, <= 2 decimals
  仰角(us)  elevation servo pulse, whole number 500-2500
  轉速(%)   Falcon command 0-100, <= 1 decimal

STRICT: those four headers must each appear exactly once, spelled exactly so.
Other columns (notes, dates...) may be added anywhere, but need a header of
their own; they are ignored. Rows may be in any order; empty rows are
skipped. Any error -> nothing is written and nothing is uploaded.

A hole with no rows uses its row's center hole (1/7 -> 4, 2/8 -> 5, 3/9 -> 6)
on the robot; if that has none either, the autos never fire at that row.

Standard library only (no openpyxl), so it runs on any Python 3.8+.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from decimal import Decimal, InvalidOperation
from xml.sax.saxutils import escape

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
XLSX = os.path.join(ROOT, "tools", "shot_table", "shot_table.xlsx")
HEADER = os.path.join(ROOT, "src", "Constants", "ShotTableData.h")
SHEET = "射擊表"
COLS = {"hole": "洞口", "dist": "距離(m)", "elev": "仰角(us)", "fly": "轉速(%)"}

NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
      "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
      "rel": "http://schemas.openxmlformats.org/package/2006/relationships"}


class TableError(Exception):
    pass


# ----------------------------------------------------------------- reading

def col_letters(ref):
    return re.match(r"([A-Z]+)", ref).group(1)


def col_index(letters):
    n = 0
    for ch in letters:
        n = n * 26 + (ord(ch) - 64)
    return n


def read_sheet(path, sheet_name):
    """{(row, col): text} for one sheet, all values as strings."""
    if not os.path.exists(path):
        raise TableError(f"找不到 Excel：{path}")
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        raise TableError(f"{path} 不是 .xlsx 檔（請用 Excel 另存為 .xlsx）")
    with z:
        names = set(z.namelist())
        shared = []
        if "xl/sharedStrings.xml" in names:
            for si in ET.fromstring(z.read("xl/sharedStrings.xml")).findall("m:si", NS):
                shared.append("".join(t.text or "" for t in si.iter(f"{{{NS['m']}}}t")))
        wb = ET.fromstring(z.read("xl/workbook.xml"))
        rels = ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))
        target = None
        for sh in wb.find("m:sheets", NS):
            if sh.get("name") == sheet_name:
                rid = sh.get(f"{{{NS['r']}}}id")
                for rel in rels:
                    if rel.get("Id") == rid:
                        target = rel.get("Target")
        if target is None:
            raise TableError(f"Excel 裡沒有名為「{sheet_name}」的工作表（名稱不可改）")
        target = target.lstrip("/")
        if not target.startswith("xl/"):
            target = "xl/" + target
        root = ET.fromstring(z.read(target))
        if root.find("m:mergeCells", NS) is not None:
            raise TableError(f"「{sheet_name}」裡有合併儲存格，請取消合併")
        cells = {}
        for c in root.iter(f"{{{NS['m']}}}c"):
            ref = c.get("r")
            m = re.match(r"([A-Z]+)(\d+)$", ref or "")
            if not m:
                continue
            t = c.get("t")
            v = c.find("m:v", NS)
            if t == "s":
                text = shared[int(v.text)] if v is not None else ""
            elif t == "inlineStr":
                text = "".join(x.text or "" for x in c.iter(f"{{{NS['m']}}}t"))
            elif t == "e":
                raise TableError(f"{ref} 是錯誤值（#REF!/#VALUE! 等），請修正")
            else:
                text = v.text if v is not None else ""
            text = (text or "").strip()
            if text != "":
                cells[(int(m.group(2)), col_index(m.group(1)))] = text
        return cells


def cell_name(row, col):
    letters = ""
    while col:
        col, rem = divmod(col - 1, 26)
        letters = chr(65 + rem) + letters
    return f"{letters}{row}"


def number(text, where, what):
    try:
        d = Decimal(text)
    except InvalidOperation:
        raise TableError(f"{where}：{what}要是數字，現在是「{text}」")
    if not d.is_finite():
        raise TableError(f"{where}：{what}要是數字，現在是「{text}」")
    # A spreadsheet may store 2.35 as 2.3500000000000001: take the shortest
    # decimal that is the same double, so the decimals check sees "2.35".
    return Decimal(repr(float(d)))


def decimals(d):
    exp = d.normalize().as_tuple().exponent
    return max(0, -exp)


def parse_points(cells):
    if not cells:
        raise TableError(f"「{SHEET}」是空的")
    header = {c: v for (r, c), v in cells.items() if r == 1}
    errors = []
    found = {}
    for key, name in COLS.items():
        cols = [c for c, v in header.items() if v == name]
        if len(cols) == 0:
            near = [v for v in header.values() if name.split("(")[0] in v]
            hint = f"（看到「{near[0]}」，名稱必須完全是「{name}」）" if near else ""
            errors.append(f"第 1 列缺少欄位「{name}」{hint}")
        elif len(cols) > 1:
            errors.append(f"欄位「{name}」重複出現在 {', '.join(cell_name(1, c) for c in cols)}")
        else:
            found[key] = cols[0]
    seen = {}
    for c, v in header.items():
        if v in seen:
            if v not in COLS.values():
                errors.append(f"欄位名稱「{v}」重複（{cell_name(1, seen[v])}、{cell_name(1, c)}）")
        seen[v] = c
    data_cols = {c for (r, c) in cells if r > 1}
    for c in sorted(data_cols - set(header)):
        errors.append(f"{cell_name(1, c)} 沒有欄位名稱，但這一欄下面有資料（新增的欄位一定要有名稱）")
    if errors:
        raise TableError("\n".join(errors))

    points = {}
    last_row = max(r for (r, c) in cells)
    for row in range(2, last_row + 1):
        vals = {k: cells.get((row, c)) for k, c in found.items()}
        if all(v is None for v in vals.values()):
            if any((row, c) in cells for c in data_cols):
                errors.append(f"第 {row} 列只有其他欄位有資料，四個必要欄位都是空的")
            continue
        missing = [COLS[k] for k, v in vals.items() if v is None]
        if missing:
            errors.append(f"第 {row} 列缺少：{'、'.join(missing)}")
            continue
        try:
            hole = number(vals["hole"], cell_name(row, found["hole"]), "洞口")
            if hole != hole.to_integral_value() or not 1 <= hole <= 9:
                raise TableError(f"{cell_name(row, found['hole'])}：洞口要是 1–9 的整數，現在是「{vals['hole']}」")
            dist = number(vals["dist"], cell_name(row, found["dist"]), "距離")
            if not Decimal("0.5") <= dist <= Decimal("15") or decimals(dist) > 2:
                raise TableError(f"{cell_name(row, found['dist'])}：距離要在 0.5–15 m、最多 2 位小數，現在是「{vals['dist']}」")
            elev = number(vals["elev"], cell_name(row, found["elev"]), "仰角")
            if elev != elev.to_integral_value() or not 500 <= elev <= 2500:
                raise TableError(f"{cell_name(row, found['elev'])}：仰角要是 500–2500 的整數 µs，現在是「{vals['elev']}」")
            fly = number(vals["fly"], cell_name(row, found["fly"]), "轉速")
            if not 0 <= fly <= 100 or decimals(fly) > 1:
                raise TableError(f"{cell_name(row, found['fly'])}：轉速要在 0–100 %、最多 1 位小數，現在是「{vals['fly']}」")
        except TableError as e:
            errors.append(str(e))
            continue
        h, cm = int(hole), int(dist * 100)
        if cm in points.setdefault(h, {}):
            errors.append(f"第 {row} 列：洞口 {h} 的 {dist} m 重複了（第 {points[h][cm][3]} 列已經有）")
            continue
        points[h][cm] = (cm, int(fly * 10), int(elev), row)
    if errors:
        raise TableError("\n".join(errors))
    if not points:
        raise TableError(f"「{SHEET}」沒有任何資料列")
    return {h: [p[:3] for _, p in sorted(pts.items())] for h, pts in points.items()}


# ----------------------------------------------------------------- header

FALLBACK = {1: 4, 7: 4, 2: 5, 8: 5, 3: 6, 9: 6}


def render_header(points):
    lines = [
        "// GENERATED by tools/shot_table/apply.py from tools/shot_table/shot_table.xlsx.",
        "// DO NOT EDIT BY HAND: change the Excel and run",
        "//   python3 tools/shot_table/apply.py --upload",
        "//",
        "// One table per Dashboard hole (key 1-9): camera distance (cm) -> Falcon",
        "// command (per mille) and elevation pulse (us), ascending by distance.",
        "// A hole with no points uses its row's center hole (TuningConstants::",
        "// ShotTable::tableForVisionId). Integers keep it small in the Mega's RAM.",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "namespace ShotTableData {",
        "",
        "struct Point {",
        "    uint16_t distanceCm;",
        "    uint16_t flywheelPermille;",
        "    uint16_t elevationUs;",
        "};",
        "",
    ]
    for h in range(1, 10):
        pts = points.get(h, [])
        if not pts:
            lines.append(f"// Hole {h}: no points (uses hole {FALLBACK.get(h, h)}).")
            continue
        lines.append(f"constexpr Point HOLE_{h}[] = {{")
        for cm, permille, us in pts:
            lines.append(f"    {{{cm}, {permille}, {us}}},  // {cm / 100:.2f} m, {permille / 10:.1f}%, {us} us")
        lines.append("};")
    lines.append("")
    lines.append("// Index = Dashboard hole key (0 unused).")
    lines.append("constexpr const Point* HOLE_POINTS[10] = {")
    lines.append("    nullptr,")
    for h in range(1, 10):
        lines.append(f"    {'HOLE_' + str(h) if points.get(h) else 'nullptr'},")
    lines.append("};")
    counts = ", ".join(str(len(points.get(h, []))) for h in range(1, 10))
    lines.append(f"constexpr uint8_t HOLE_COUNTS[10] = {{0, {counts}}};")
    lines.append("")
    lines.append("}  // namespace ShotTableData")
    lines.append("")
    return "\n".join(lines)


def parse_header(text):
    points = {}
    for m in re.finditer(r"constexpr Point HOLE_(\d)\[\] = \{(.*?)\};", text, re.S):
        pts = [tuple(int(x) for x in p) for p in re.findall(r"\{(\d+), (\d+), (\d+)\}", m.group(2))]
        points[int(m.group(1))] = pts
    return points


def describe_changes(old, new):
    out = []
    for h in range(1, 10):
        o = {p[0]: p for p in old.get(h, [])}
        n = {p[0]: p for p in new.get(h, [])}
        for cm in sorted(set(o) | set(n)):
            fmt = lambda p: f"{p[2]} us / {p[1] / 10:.1f}%"
            if cm not in o:
                out.append(f"  + 洞口 {h} {cm / 100:.2f} m：{fmt(n[cm])}")
            elif cm not in n:
                out.append(f"  - 洞口 {h} {cm / 100:.2f} m：{fmt(o[cm])}")
            elif o[cm] != n[cm]:
                out.append(f"  * 洞口 {h} {cm / 100:.2f} m：{fmt(o[cm])} -> {fmt(n[cm])}")
    return out


# ----------------------------------------------------------------- export

def write_xlsx(path, points):
    def cell(ref, value, style=0):
        s = f' s="{style}"' if style else ""
        if isinstance(value, str):
            return f'<c r="{ref}" t="inlineStr"{s}><is><t>{escape(value)}</t></is></c>'
        return f'<c r="{ref}"{s}><v>{value}</v></c>'

    rows = [["洞口", "距離(m)", "仰角(us)", "轉速(%)", "備註"]]
    for h in range(1, 10):
        for cm, permille, us in points.get(h, []):
            rows.append([h, round(cm / 100, 2), us, round(permille / 10, 1), ""])
    sheet_rows = []
    for i, r in enumerate(rows, 1):
        cs = "".join(cell(cell_name(i, j), v, 1 if i == 1 else 0)
                     for j, v in enumerate(r, 1) if v != "")
        sheet_rows.append(f'<row r="{i}">{cs}</row>')
    dv = (
        '<dataValidations count="4">'
        '<dataValidation type="whole" allowBlank="1" showErrorMessage="1" errorTitle="洞口" '
        'error="洞口要是 1–9 的整數" sqref="A2:A500"><formula1>1</formula1><formula2>9</formula2></dataValidation>'
        '<dataValidation type="decimal" allowBlank="1" showErrorMessage="1" errorTitle="距離" '
        'error="距離 0.5–15 m" sqref="B2:B500"><formula1>0.5</formula1><formula2>15</formula2></dataValidation>'
        '<dataValidation type="whole" allowBlank="1" showErrorMessage="1" errorTitle="仰角" '
        'error="仰角 500–2500 的整數 µs" sqref="C2:C500"><formula1>500</formula1><formula2>2500</formula2></dataValidation>'
        '<dataValidation type="decimal" allowBlank="1" showErrorMessage="1" errorTitle="轉速" '
        'error="轉速 0–100 %" sqref="D2:D500"><formula1>0</formula1><formula2>100</formula2></dataValidation>'
        '</dataValidations>')
    sheet1 = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<sheetViews><sheetView workbookViewId="0"><pane ySplit="1" topLeftCell="A2" '
        'activePane="bottomLeft" state="frozen"/></sheetView></sheetViews>'
        '<cols><col min="1" max="4" width="12" customWidth="1"/>'
        '<col min="5" max="5" width="40" customWidth="1"/></cols>'
        f'<sheetData>{"".join(sheet_rows)}</sheetData>{dv}</worksheet>')
    help_lines = [
        "射擊表怎麼用",
        "",
        "1. 在「射擊表」工作表改數字、加列、刪列，存檔（保持 .xlsx）。",
        "2. 在專案資料夾執行：python3 tools/shot_table/apply.py --upload",
        "   會檢查格式、列出改了哪些值、產生 src/Constants/ShotTableData.h，再燒進 Mega。",
        "   只想檢查：python3 tools/shot_table/apply.py --check",
        "",
        "格式（嚴格）",
        "- 第 1 列是欄位名稱。「洞口」「距離(m)」「仰角(us)」「轉速(%)」四個必須各出現一次、名稱一字不差。",
        "- 可以在任何位置插入其他欄位（例如日期、誰測的），但新欄位一定要有名稱；它們不會被讀取。",
        "- 洞口：Dashboard 的編號 1–9（1 左下、2 左中、3 左上 … 9 右上）。",
        "- 距離：鎖定目標時 Dashboard 顯示的相機距離（不是捲尺），0.5–15 m，最多 2 位小數。",
        "- 仰角：Dashboard 的仰角脈寬，500–2500 的整數。",
        "- 轉速：Falcon 指令 0–100 %，最多 1 位小數。",
        "- 列的順序不重要；空白列會略過；同一個洞口的同一個距離不能重複。",
        "- 任何一格不符合，就不會產生檔案、也不會燒錄，畫面會指出是哪一格。",
        "",
        "沒有資料的洞",
        "- 會借用同一排中間洞的值：1、7 用 4，2、8 用 5，3、9 用 6。",
        "- 兩個距離之間直線內插；比最近的點還近就用最近的點；比最遠的點還遠就用最遠的點。",
    ]
    sheet2 = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<cols><col min="1" max="1" width="110" customWidth="1"/></cols><sheetData>'
        + "".join(f'<row r="{i}">{cell("A" + str(i), t, 1 if i == 1 else 0)}</row>'
                  for i, t in enumerate(help_lines, 1) if t)
        + "</sheetData></worksheet>")
    files = {
        "[Content_Types].xml":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
            '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
            '<Default Extension="xml" ContentType="application/xml"/>'
            '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
            '<Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
            '<Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
            '<Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
            '</Types>',
        "_rels/.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
            '</Relationships>',
        "xl/workbook.xml":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
            f'<sheets><sheet name="{SHEET}" sheetId="1" r:id="rId1"/><sheet name="說明" sheetId="2" r:id="rId2"/></sheets>'
            '</workbook>',
        "xl/_rels/workbook.xml.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>'
            '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>'
            '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
            '</Relationships>',
        "xl/styles.xml":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
            '<fonts count="2"><font><sz val="12"/><name val="Calibri"/></font>'
            '<font><b/><sz val="12"/><name val="Calibri"/></font></fonts>'
            '<fills count="3"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill>'
            '<fill><patternFill patternType="solid"><fgColor rgb="FFDDEBF7"/></patternFill></fill></fills>'
            '<borders count="1"><border/></borders>'
            '<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>'
            '<cellXfs count="2"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>'
            '<xf numFmtId="0" fontId="1" fillId="2" borderId="0" xfId="0" applyFont="1" applyFill="1"/></cellXfs>'
            '</styleSheet>',
        "xl/worksheets/sheet1.xml": sheet1,
        "xl/worksheets/sheet2.xml": sheet2,
    }
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for name, body in files.items():
            z.writestr(name, body)


# ----------------------------------------------------------------- main

def find_pio():
    for cand in (shutil.which("pio"),
                 os.path.expanduser("~/.platformio/penv/bin/pio"),
                 os.path.expanduser("~/.platformio/penv/Scripts/pio.exe")):
        if cand and os.path.exists(cand):
            return cand
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xlsx", default=XLSX)
    ap.add_argument("--check", action="store_true", help="check only, write nothing")
    ap.add_argument("--upload", action="store_true", help="also build and upload the Mega")
    ap.add_argument("--env", default="megaatmega2560")
    ap.add_argument("--export", action="store_true", help="rebuild the Excel from the current header")
    args = ap.parse_args()

    old_text = open(HEADER, encoding="utf-8").read() if os.path.exists(HEADER) else ""
    old = parse_header(old_text)

    if args.export:
        if os.path.exists(args.xlsx):
            sys.exit(f"{args.xlsx} 已存在；要重建請先自己移走它（不會覆蓋）")
        write_xlsx(args.xlsx, old)
        print(f"已從 {os.path.relpath(HEADER, ROOT)} 產生 {os.path.relpath(args.xlsx, ROOT)}")
        return

    try:
        points = parse_points(read_sheet(args.xlsx, SHEET))
    except TableError as e:
        print("格式錯誤，沒有寫入任何檔案、也沒有燒錄：\n" + str(e), file=sys.stderr)
        sys.exit(1)

    total = sum(len(p) for p in points.values())
    print(f"讀到 {total} 個點：" + "，".join(f"洞口 {h} {len(points[h])} 點" for h in sorted(points)))
    missing = [h for h in range(1, 10) if h not in points]
    for h in missing:
        src = FALLBACK.get(h)
        note = f"借用洞口 {src}" if src in points else "同排中間洞也沒有 -> 自動模式不會對它發射"
        print(f"  洞口 {h} 沒有資料：{note}")
    changes = describe_changes(old, points)
    print("改動：" if changes else "跟目前的射擊表相同，沒有改動。")
    for line in changes:
        print(line)

    if args.check:
        return
    new_text = render_header(points)
    if new_text != old_text:
        with open(HEADER, "w", encoding="utf-8", newline="\n") as f:
            f.write(new_text)
        print(f"已寫入 {os.path.relpath(HEADER, ROOT)}")
    if args.upload:
        pio = find_pio()
        if pio is None:
            sys.exit("找不到 PlatformIO（pio）。請先安裝，或用 VS Code 的 PlatformIO 燒錄。")
        print(f"編譯並燒錄 Mega（{args.env}）…")
        sys.exit(subprocess.call([pio, "run", "-e", args.env, "-t", "upload"], cwd=ROOT))


if __name__ == "__main__":
    main()
