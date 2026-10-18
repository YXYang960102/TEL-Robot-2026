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
  轉速      Falcon command, in the unit of the next column
  轉速單位  "%" (0-100, <= 1 decimal) or "us" (the Dashboard's Falcon PWM
            pulse, whole number, on the forward side of 1500 -- 1000-1500
            while Flywheel::INVERTED is true; converted to %)

STRICT: those five headers must each appear exactly once, spelled exactly so.
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
COLS = {"hole": "洞口", "dist": "距離(m)", "elev": "仰角(us)", "fly": "轉速", "unit": "轉速單位"}
UNITS = ("%", "us")
TUNING = os.path.join(ROOT, "src", "Constants", "TuningConstants.h")


def flywheel_inverted():
    """Flywheel::INVERTED from TuningConstants.h: which side of 1500 us is
    the shooting direction (true = below), for the "us" unit."""
    text = open(TUNING, encoding="utf-8").read()
    m = re.search(r"namespace Flywheel \{.*?constexpr bool INVERTED = (true|false);", text, re.S)
    if not m:
        raise TableError("讀不到 TuningConstants.h 的 Flywheel::INVERTED，無法換算 us")
    return m.group(1) == "true"

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
    inverted = None  # read TuningConstants only if a row uses "us"
    last_row = max(r for (r, c) in cells)
    for row in range(2, last_row + 1):
        vals = {k: cells.get((row, c)) for k, c in found.items()}
        if all(v is None for v in vals.values()):
            if any((row, c) in cells for c in data_cols):
                errors.append(f"第 {row} 列只有其他欄位有資料，必要欄位都是空的")
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
            unit = vals["unit"]
            uref = cell_name(row, found["unit"])
            if unit not in UNITS:
                raise TableError(f"{uref}：轉速單位只能是「%」或「us」（用下拉選單），現在是「{unit}」")
            fref = cell_name(row, found["fly"])
            fly = number(vals["fly"], fref, "轉速")
            if unit == "%":
                if not 0 <= fly <= 100 or decimals(fly) > 1:
                    raise TableError(f"{fref}：單位 % 時轉速要在 0–100、最多 1 位小數，現在是「{vals['fly']}」")
                permille = int(fly * 10)
            else:
                if inverted is None:
                    inverted = flywheel_inverted()
                lo, hi = (1000, 1500) if inverted else (1500, 2000)
                if fly != fly.to_integral_value() or not lo <= fly <= hi:
                    side = "低於" if inverted else "高於"
                    raise TableError(f"{fref}：單位 us 時轉速要是 {lo}–{hi} 的整數（正轉方向是{side} 1500），現在是「{vals['fly']}」")
                # 500 us from neutral = 100% -> 1 us = 2 per mille.
                permille = int(abs(int(fly) - 1500) * 2)
        except TableError as e:
            errors.append(str(e))
            continue
        h, cm = int(hole), int(dist * 100)
        if cm in points.setdefault(h, {}):
            errors.append(f"第 {row} 列：洞口 {h} 的 {dist} m 重複了（第 {points[h][cm][3]} 列已經有）")
            continue
        points[h][cm] = (cm, permille, int(elev), row)
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

HELPER = "圖表資料"   # hidden sheet: per-hole points sorted by distance, for the charts
MAX_ROWS = 500         # data rows the formulas / validations cover (2..MAX_ROWS)
SORTED_ROWS = 60       # points per hole the charts can show
SERIES_COLORS = ["1F77B4", "FF7F0E", "2CA02C", "D62728", "9467BD",
                 "8C564B", "E377C2", "7F7F7F", "BCBD22"]


def _q(sheet):
    return "'" + sheet.replace("'", "''") + "'"


CHART_HOLES = [(4, "下排（洞口 4，約 120 cm）"), (5, "中排（洞口 5，約 180 cm）"),
               (6, "上排（洞口 6，約 240 cm）")]   # the center column, one chart per height


def _height_chart_xml(slot, title):
    """One height: flywheel % (points + power trendline, y = a*x^b, through
    the origin) on the left axis, elevation us (points + lines) on the
    right, against the distance to the target. Reads helper columns
    slot*3+1 .. slot*3+3 (distance, elevation, flywheel), sorted."""
    base = 1 + slot * 3
    col = lambda k: cell_name(1, base + k)[:-1]
    rng = lambda c: f"{_q(HELPER)}!${c}$2:${c}${SORTED_ROWS + 1}"

    def rich(text, size=1200, bold=1):
        return (f'<c:tx><c:rich><a:bodyPr/><a:p><a:pPr><a:defRPr sz="{size}" b="{bold}"/></a:pPr>'
                f'<a:r><a:rPr lang="zh-TW" sz="{size}" b="{bold}"/><a:t>{escape(text)}</a:t></a:r></a:p></c:rich></c:tx>')

    def line(color, width=22225, dash=None, none=False):
        if none:
            return '<a:ln><a:noFill/></a:ln>'
        d = f'<a:prstDash val="{dash}"/>' if dash else ""
        return f'<a:ln w="{width}"><a:solidFill><a:srgbClr val="{color}"/></a:solidFill>{d}</a:ln>'

    def ser(idx, name, color, ycol, trend, connect):
        marker = (f'<c:marker><c:symbol val="circle"/><c:size val="7"/>'
                  f'<c:spPr><a:solidFill><a:srgbClr val="{color}"/></a:solidFill></c:spPr></c:marker>')
        tl = (f'<c:trendline><c:name>{escape(name)} 趨勢（乘冪）</c:name><c:spPr>{line(color, 25400)}</c:spPr>'
              '<c:trendlineType val="power"/><c:dispRSqr val="0"/><c:dispEq val="0"/></c:trendline>') if trend else ""
        return (f'<c:ser><c:idx val="{idx}"/><c:order val="{idx}"/><c:tx><c:v>{escape(name)}</c:v></c:tx>'
                f'<c:spPr>{line(color, dash="sysDash") if connect else line(color, none=True)}</c:spPr>'
                f'{marker}{tl}'
                f'<c:xVal><c:numRef><c:f>{rng(col(0))}</c:f></c:numRef></c:xVal>'
                f'<c:yVal><c:numRef><c:f>{rng(ycol)}</c:f></c:numRef></c:yVal><c:smooth val="0"/></c:ser>')

    def axis(ax_id, cross, pos, title_text, lo, hi, delete=False, crosses="autoZero", grid=True):
        scale = '<c:orientation val="minMax"/>'
        if hi is not None:
            scale += f'<c:max val="{hi}"/>'
        if lo is not None:
            scale += f'<c:min val="{lo}"/>'
        ttl = f'<c:title>{rich(title_text, 1000, 0)}<c:overlay val="0"/></c:title>' if title_text else ""
        return (f'<c:valAx><c:axId val="{ax_id}"/><c:scaling>{scale}</c:scaling>'
                f'<c:delete val="{1 if delete else 0}"/><c:axPos val="{pos}"/>'
                + ('<c:majorGridlines/>' if grid else '') + ttl +
                f'<c:numFmt formatCode="General" sourceLinked="0"/><c:tickLblPos val="nextTo"/>'
                f'<c:crossAx val="{cross}"/><c:crosses val="{crosses}"/><c:crossBetween val="midCat"/></c:valAx>')

    fly = ('<c:scatterChart><c:scatterStyle val="lineMarker"/><c:varyColors val="0"/>'
           + ser(0, "轉速 (%)", "D62728", col(2), trend=True, connect=False)
           + '<c:axId val="510000001"/><c:axId val="510000002"/></c:scatterChart>')
    elev = ('<c:scatterChart><c:scatterStyle val="lineMarker"/><c:varyColors val="0"/>'
            + ser(1, "仰角 (µs)", "1F77B4", col(1), trend=False, connect=True)
            + '<c:axId val="510000003"/><c:axId val="510000004"/></c:scatterChart>')
    return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<c:chartSpace xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" '
            'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
            '<c:roundedCorners val="0"/>'
            f'<c:chart><c:title>{rich(title + " 射擊曲線")}<c:overlay val="0"/></c:title><c:autoTitleDeleted val="0"/>'
            '<c:plotArea><c:layout/>' + fly + elev
            + axis(510000001, 510000002, "b", "到目標距離 (m)", 0, None)
            + axis(510000002, 510000001, "l", "轉速 (%)", 0, 100)
            + axis(510000003, 510000004, "b", None, 0, None, delete=True, grid=False)
            + axis(510000004, 510000003, "r", "仰角 (µs)", 500, 2500, crosses="max", grid=False)
            + '</c:plotArea><c:legend><c:legendPos val="b"/><c:overlay val="0"/></c:legend>'
            '<c:plotVisOnly val="0"/><c:dispBlanksAs val="gap"/></c:chart></c:chartSpace>')


def _anchor(idx, col0, row0, col1, row1, rid, name):
    return (f'<xdr:twoCellAnchor editAs="oneCell">'
            f'<xdr:from><xdr:col>{col0}</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>{row0}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from>'
            f'<xdr:to><xdr:col>{col1}</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>{row1}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:to>'
            f'<xdr:graphicFrame macro=""><xdr:nvGraphicFramePr><xdr:cNvPr id="{idx + 1}" name="{name}"/><xdr:cNvGraphicFramePr/></xdr:nvGraphicFramePr>'
            f'<xdr:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/></xdr:xfrm>'
            f'<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/chart">'
            f'<c:chart xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" r:id="{rid}"/>'
            f'</a:graphicData></a:graphic></xdr:graphicFrame><xdr:clientData/></xdr:twoCellAnchor>')


def write_xlsx(path, points):
    def cell(ref, value, style=0):
        s = f' s="{style}"' if style else ""
        if isinstance(value, str):
            return f'<c r="{ref}" t="inlineStr"{s}><is><t>{escape(value)}</t></is></c>'
        return f'<c r="{ref}"{s}><v>{value}</v></c>'

    # Sheet 1: the table the operator edits (only the five columns).
    rows = [[COLS["hole"], COLS["dist"], COLS["elev"], COLS["fly"], COLS["unit"]]]
    for h in range(1, 10):
        for cm, permille, us in points.get(h, []):
            rows.append([h, round(cm / 100, 2), us, round(permille / 10, 1), "%"])
    sheet_rows = []
    for i, r in enumerate(rows, 1):
        cs = "".join(cell(cell_name(i, j), v, 1 if i == 1 else 0) for j, v in enumerate(r, 1))
        sheet_rows.append(f'<row r="{i}">{cs}</row>')
    dv = (
        '<dataValidations count="5">'
        '<dataValidation type="whole" allowBlank="1" showErrorMessage="1" errorTitle="洞口" '
        f'error="洞口要是 1–9 的整數" sqref="A2:A{MAX_ROWS}"><formula1>1</formula1><formula2>9</formula2></dataValidation>'
        '<dataValidation type="decimal" allowBlank="1" showErrorMessage="1" errorTitle="距離" '
        f'error="距離 0.5–15 m" sqref="B2:B{MAX_ROWS}"><formula1>0.5</formula1><formula2>15</formula2></dataValidation>'
        '<dataValidation type="whole" allowBlank="1" showErrorMessage="1" errorTitle="仰角" '
        f'error="仰角 500–2500 的整數 µs" sqref="C2:C{MAX_ROWS}"><formula1>500</formula1><formula2>2500</formula2></dataValidation>'
        '<dataValidation type="custom" allowBlank="1" showErrorMessage="1" errorTitle="轉速" '
        'error="單位 % 時 0–100；單位 us 時 1000–2000 的整數" '
        f'sqref="D2:D{MAX_ROWS}"><formula1>{escape("OR(AND(E2=\"%\",D2>=0,D2<=100),AND(E2=\"us\",D2>=1000,D2<=2000,INT(D2)=D2))")}</formula1></dataValidation>'
        '<dataValidation type="list" allowBlank="1" showErrorMessage="1" errorTitle="轉速單位" '
        f'error="只能選 % 或 us" sqref="E2:E{MAX_ROWS}"><formula1>"%,us"</formula1></dataValidation>'
        '</dataValidations>')
    sheet1 = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        '<sheetViews><sheetView tabSelected="1" workbookViewId="0"><pane ySplit="1" topLeftCell="A2" '
        'activePane="bottomLeft" state="frozen"/></sheetView></sheetViews>'
        '<cols><col min="1" max="5" width="11" customWidth="1"/></cols>'
        f'<sheetData>{"".join(sheet_rows)}</sheetData>{dv}<drawing r:id="rId1"/></worksheet>')

    # Sheet 3 (hidden): for each hole, its points sorted by distance --
    # distance / elevation / flywheel -- as formulas over sheet 1, so the
    # charts follow every edit, added row and deleted row by themselves.
    t = _q(SHEET)
    hole_rng = f"{t}!$A$2:$A${MAX_ROWS}"
    dist_rng = f"{t}!$B$2:$B${MAX_ROWS}"
    helper_rows = []
    head = []
    for slot, (h, _) in enumerate(CHART_HOLES):
        base = 1 + slot * 3
        for k, label in enumerate((f"洞口{h} 距離", f"洞口{h} 仰角", f"洞口{h} 轉速")):
            head.append(cell(cell_name(1, base + k), label, 1))
    helper_rows.append(f'<row r="1">{"".join(head)}</row>')
    for n in range(1, SORTED_ROWS + 1):
        r = n + 1
        cs = []
        for slot, (h, _) in enumerate(CHART_HOLES):
            base = 1 + slot * 3
            dref = cell_name(r, base)
            fd = f"IFERROR(SMALL(IF({hole_rng}={h},{dist_rng}),{n}),NA())"
            cs.append(f'<c r="{dref}"><f t="array" ref="{dref}">{escape(fd)}</f></c>')
            match = f"MATCH(1,({hole_rng}={h})*({dist_rng}={dref}),0)"
            fly_pct = (f'IF({t}!$E$2:$E${MAX_ROWS}="us",ABS({t}!$D$2:$D${MAX_ROWS}-1500)/5,'
                       f'{t}!$D$2:$D${MAX_ROWS})')  # "us" rows shown as % too
            for k, src in ((1, f"{t}!$C$2:$C${MAX_ROWS}"), (2, fly_pct)):
                ref = cell_name(r, base + k)
                f = f"IFERROR(INDEX({src},{match}),NA())"
                cs.append(f'<c r="{ref}"><f t="array" ref="{ref}">{escape(f)}</f></c>')
        helper_rows.append(f'<row r="{r}">{"".join(cs)}</row>')
    sheet3 = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        f'<sheetData>{"".join(helper_rows)}</sheetData></worksheet>')

    help_lines = [
        "射擊表怎麼用",
        "",
        "1. 在「射擊表」工作表改數字、加列、刪列，存檔（保持 .xlsx）。旁邊三張射擊曲線圖（下、中、上排，用中間那一行的洞口 4、5、6）會自動跟著更新。",
        "2. 在專案資料夾執行：python3 tools/shot_table/apply.py --upload",
        "   會檢查格式、列出改了哪些值、產生 src/Constants/ShotTableData.h，再燒進 Mega。",
        "   只想檢查：python3 tools/shot_table/apply.py --check",
        "",
        "格式（嚴格）",
        "- 第 1 列是欄位名稱。「洞口」「距離(m)」「仰角(us)」「轉速」「轉速單位」五個必須各出現一次、名稱一字不差。",
        "- 若插入其他欄位，新欄位一定要有名稱；它們不會被讀取。",
        "- 洞口：Dashboard 的編號 1–9（1 左下、2 左中、3 左上 … 9 右上）。",
        "- 距離：鎖定目標時 Dashboard 顯示的相機距離（不是捲尺），0.5–15 m，最多 2 位小數。",
        "- 仰角：Dashboard 的仰角脈寬，500–2500 的整數。",
        "- 轉速單位：用下拉選單選「%」或「us」，每一列可以不同。",
        "- 轉速：單位 % 時 0–100、最多 1 位小數；單位 us 時填 Dashboard「Falcon PWM」的脈寬整數，必須在正轉那一側（目前是 1000–1500），會自動換算成 %。",
        "- 列的順序不重要；空白列會略過；同一個洞口的同一個距離不能重複。",
        f"- 資料請放在第 2–{MAX_ROWS} 列；每個洞口最多 {SORTED_ROWS} 個點會畫進曲線圖。",
        "- 任何一格不符合，就不會產生檔案、也不會燒錄，畫面會指出是哪一格。",
        "",
        "沒有資料的洞",
        "- 會借用同一排中間洞的值：1、7 用 4，2、8 用 5，3、9 用 6。",
        "- Mega 在兩個量測點之間用直線內插；比最近的點還近就用最近的點；比最遠的點還遠就用最遠的點。",
        "- 曲線圖上的轉速曲線是 Excel 的乘冪趨勢線（y = a·x^b，從原點出發），只是用來看資料合不合理、哪個點量歪了；Mega 不是照這條曲線打。",
        "",
        "「圖表資料」是隱藏的工作表，由公式自動整理給曲線圖用，不用動它。",
    ]
    sheet2 = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<cols><col min="1" max="1" width="120" customWidth="1"/></cols><sheetData>'
        + "".join(f'<row r="{i}">{cell("A" + str(i), line, 1 if i == 1 else 0)}</row>'
                  for i, line in enumerate(help_lines, 1) if line)
        + "</sheetData></worksheet>")

    drawing = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<xdr:wsDr xmlns:xdr="http://schemas.openxmlformats.org/drawingml/2006/spreadsheetDrawing" '
        'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        + "".join(_anchor(i + 1, 5, i * 20, 15, i * 20 + 19, f"rId{i + 1}", f"射擊曲線{i + 1}")
                  for i in range(len(CHART_HOLES)))
        + '</xdr:wsDr>')
    ct = "application/vnd.openxmlformats-officedocument"
    files = {
        "[Content_Types].xml":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
            '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
            '<Default Extension="xml" ContentType="application/xml"/>'
            f'<Override PartName="/xl/workbook.xml" ContentType="{ct}.spreadsheetml.sheet.main+xml"/>'
            f'<Override PartName="/xl/worksheets/sheet1.xml" ContentType="{ct}.spreadsheetml.worksheet+xml"/>'
            f'<Override PartName="/xl/worksheets/sheet2.xml" ContentType="{ct}.spreadsheetml.worksheet+xml"/>'
            f'<Override PartName="/xl/worksheets/sheet3.xml" ContentType="{ct}.spreadsheetml.worksheet+xml"/>'
            f'<Override PartName="/xl/styles.xml" ContentType="{ct}.spreadsheetml.styles+xml"/>'
            f'<Override PartName="/xl/drawings/drawing1.xml" ContentType="{ct}.drawing+xml"/>'
            + "".join(f'<Override PartName="/xl/charts/chart{i + 1}.xml" ContentType="{ct}.drawingml.chart+xml"/>'
                      for i in range(len(CHART_HOLES))) +
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
            f'<sheets><sheet name="{SHEET}" sheetId="1" r:id="rId1"/><sheet name="說明" sheetId="2" r:id="rId2"/>'
            f'<sheet name="{HELPER}" sheetId="3" state="hidden" r:id="rId3"/></sheets>'
            '<calcPr calcId="191029" fullCalcOnLoad="1"/>'
            '</workbook>',
        "xl/_rels/workbook.xml.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>'
            '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>'
            '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet3.xml"/>'
            '<Relationship Id="rId4" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
            '</Relationships>',
        "xl/worksheets/_rels/sheet1.xml.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/drawing" Target="../drawings/drawing1.xml"/>'
            '</Relationships>',
        "xl/drawings/_rels/drawing1.xml.rels":
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            + "".join(f'<Relationship Id="rId{i + 1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/chart" Target="../charts/chart{i + 1}.xml"/>'
                      for i in range(len(CHART_HOLES))) +
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
            '<cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>'
            '</styleSheet>',
        "xl/worksheets/sheet1.xml": sheet1,
        "xl/worksheets/sheet2.xml": sheet2,
        "xl/worksheets/sheet3.xml": sheet3,
        "xl/drawings/drawing1.xml": drawing,
    }
    for i, (_, title) in enumerate(CHART_HOLES):
        files[f"xl/charts/chart{i + 1}.xml"] = _height_chart_xml(i, title)
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
