#!/usr/bin/env python3
"""Shot table: Excel -> src/Constants/ShotTableData.h -> (optionally) the Mega.

  python3 tools/shot_table/apply.py            check the Excel, show what changes, write the header
  python3 tools/shot_table/apply.py --upload   ...then build and upload the Mega (pio)
  python3 tools/shot_table/apply.py --check    check only, write nothing
  python3 tools/shot_table/apply.py --export   rebuild shot_table.xlsx from the current header
  python3 tools/shot_table/apply.py --replace [CSV] [--all-colors] [--upload]
        REPLACE (2026-11-16): the color sheet(s) the CSV's 球色 column names
        become exactly the CSV's points -- every older point of that color
        is gone (old and new measurements never mix). --all-colors: the
        same points into all six sheets.
  python3 tools/shot_table/apply.py --adjust [CSV] [--near 0.3] [--upload]
        ADJUST (2026-11-16): each CSV point overwrites the point of the same
        color and hole whose distance is nearest within --near meters (0.3
        default; e.g. 4.56 m replaces 4.47 m, distance included); none that
        close -> added as a new point. Everything else stays.
        --import is the same as --adjust.
  No CSV given = the newest shot_log_*.csv in ~/Downloads. With --check
  either only shows what it would change.

The Excel (tools/shot_table/shot_table.xlsx) has one sheet per ball color
(2026-11-15): "射擊表-黃" "射擊表-紫" "射擊表-藍" "射擊表-綠" "射擊表-紅"
"射擊表-橘" -- the Dashboard's ball color picks which table the Mega uses.
A color sheet with no rows uses the yellow table. In each, row 1 = headers,
one measured point per row:

  洞口      Dashboard key 1-9 (1 left-bottom, 2 left-mid, 3 left-top ... 9 right-top),
            10 / 11 / 12 = the bonus holes left / middle / right (keys Q W E)
  距離(m)   the camera distance shown on the Dashboard while locked, 0.5-15 m, <= 2 decimals
  仰角(us)  elevation servo pulse, whole number 500-2461 (Angle::MIN/MAX_PULSE_US)
  轉速      Falcon command, in the unit of the next column
  轉速單位  "%" (0-100, <= 1 decimal) or "us" (the Dashboard's Falcon PWM
            pulse, whole number, on the forward side of 1500: 1500-2000
            when Flywheel::INVERTED is false, 1000-1500 when true -- read
            from TuningConstants.h; converted to %)

STRICT: those five headers must each appear exactly once, spelled exactly so.
Other columns (notes, dates...) may be added anywhere, but need a header of
their own; they are ignored. Rows may be in any order; empty rows are
skipped. Any error -> nothing is written and nothing is uploaded.

A hole with no rows uses its row's center hole (1/7 -> 4, 2/8 -> 5,
3/9 -> 6, the bonus row 10/12 -> 11; 2026-10-31) on the robot; if that has
none either, the autos never fire at it.

Standard library only (no openpyxl), so it runs on any Python 3.8+.
"""

import argparse
import csv
import glob
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import time
import zipfile
from decimal import Decimal, InvalidOperation
from xml.sax.saxutils import escape

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
XLSX = os.path.join(ROOT, "tools", "shot_table", "shot_table.xlsx")
HEADER = os.path.join(ROOT, "src", "Constants", "ShotTableData.h")
SHEET = "射擊表"   # before 2026-11-15: one sheet; now one per ball color (SHEETS)
# Ball colors (2026-11-15), in the order the Mega / radio number them
# (RCMAP_BALL_*): one sheet and one table each.
COLORS = ["黃", "紫", "藍", "綠", "紅", "橘"]
COLOR_NAMES_EN = ["yellow", "purple", "blue", "green", "red", "orange"]
SHEETS = [f"射擊表-{c}" for c in COLORS]
COLOR_COL = "球色"   # the Dashboard CSV's color column (2026-11-15)
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


def sheet_part(z, sheet_name):
    """The zip member name of one sheet (e.g. xl/worksheets/sheet1.xml)."""
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
    return target


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
        target = sheet_part(z, sheet_name)
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


def parse_rows(cells, what=None):
    """Every data row, checked: (found columns, [(row, hole, cm, permille,
    elevation us)]). Any error raises TableError with all of them."""
    what = what or f"「{SHEET}」"
    if not cells:
        raise TableError(f"{what}是空的")
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

    rows = []
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
            if hole != hole.to_integral_value() or not 1 <= hole <= 12:
                raise TableError(f"{cell_name(row, found['hole'])}：洞口要是 1–12 的整數（10–12 是加分框左中右），現在是「{vals['hole']}」")
            dist = number(vals["dist"], cell_name(row, found["dist"]), "距離")
            if not Decimal("0.5") <= dist <= Decimal("15") or decimals(dist) > 2:
                raise TableError(f"{cell_name(row, found['dist'])}：距離要在 0.5–15 m、最多 2 位小數，現在是「{vals['dist']}」")
            elev = number(vals["elev"], cell_name(row, found["elev"]), "仰角")
            if elev != elev.to_integral_value() or not 500 <= elev <= 2461:
                raise TableError(f"{cell_name(row, found['elev'])}：仰角要是 500–2461 的整數 µs，現在是「{vals['elev']}」")
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
        rows.append((row, int(hole), int(dist * 100), permille, int(elev)))
    if errors:
        raise TableError("\n".join(errors))
    if not rows:
        raise TableError(f"{what}沒有任何資料列")
    return found, rows


def parse_points(cells):
    _, rows = parse_rows(cells)
    points = {}
    errors = []
    for row, h, cm, permille, elev in rows:
        if cm in points.setdefault(h, {}):
            errors.append(f"第 {row} 列：洞口 {h} 的 {cm / 100:g} m 重複了（第 {points[h][cm][3]} 列已經有）")
            continue
        points[h][cm] = (cm, permille, elev, row)
    if errors:
        raise TableError("\n".join(errors))
    return {h: [p[:3] for _, p in sorted(pts.items())] for h, pts in points.items()}


# ----------------------------------------------------------------- header

FALLBACK = {1: 4, 7: 4, 2: 5, 8: 5, 3: 6, 9: 6, 10: 11, 12: 11}
HOLES = range(1, 13)       # 1-9 the grid, 10-12 the bonus holes (Q W E)
BONUS = {10: "加分左", 11: "加分中", 12: "加分右"}


def render_header(by_color):
    """by_color = [points of color 0..5]; an empty color uses yellow's."""
    lines = [
        "// GENERATED by tools/shot_table/apply.py from tools/shot_table/shot_table.xlsx.",
        "// DO NOT EDIT BY HAND: change the Excel and run",
        "//   python3 tools/shot_table/apply.py --upload",
        "//",
        "// One table per ball color (2026-11-15; sheets 射擊表-黃 ... 射擊表-橘, in",
        "// RCMAP_BALL_* order) and Dashboard hole (keys 1-9; 10-12 = bonus Q W E):",
        "// camera distance (cm) -> Falcon command (per mille) and elevation pulse",
        "// (us), ascending by distance. A hole with no points uses its row's center",
        "// hole (TuningConstants::ShotTable::tableForVisionId); a color with no",
        "// points at all uses yellow's. Integers keep it small in the Mega's RAM.",
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
        f"constexpr uint8_t COLOR_COUNT = {len(COLORS)};",
        "",
    ]
    source = [c if any(by_color[c].values()) else 0 for c in range(len(COLORS))]
    for c, zh in enumerate(COLORS):
        points = by_color[c]
        if source[c] != c:
            lines.append(f"// Color {c} ({COLOR_NAMES_EN[c]}, {zh}): no points -> uses yellow's table.")
            continue
        for h in HOLES:
            pts = points.get(h, [])
            if not pts:
                continue
            lines.append(f"constexpr Point C{c}_HOLE_{h}[] = {{  // {COLOR_NAMES_EN[c]}")
            for cm, permille, us in pts:
                lines.append(f"    {{{cm}, {permille}, {us}}},  // {cm / 100:.2f} m, {permille / 10:.1f}%, {us} us")
            lines.append("};")
        lines.append("")
    lines.append("// [color][Dashboard hole key] (hole 0 unused).")
    lines.append(f"constexpr const Point* HOLE_POINTS[COLOR_COUNT][13] = {{")
    for c in range(len(COLORS)):
        src = source[c]
        refs = ", ".join(f"C{src}_HOLE_{h}" if by_color[src].get(h) else "nullptr" for h in HOLES)
        lines.append(f"    {{nullptr, {refs}}},  // {COLOR_NAMES_EN[c]}")
    lines.append("};")
    lines.append(f"constexpr uint8_t HOLE_COUNTS[COLOR_COUNT][13] = {{")
    for c in range(len(COLORS)):
        src = source[c]
        counts = ", ".join(str(len(by_color[src].get(h, []))) for h in HOLES)
        lines.append(f"    {{0, {counts}}},  // {COLOR_NAMES_EN[c]}")
    lines.append("};")
    lines.append("")
    lines.append("}  // namespace ShotTableData")
    lines.append("")
    return "\n".join(lines)


def parse_header(text):
    """[points of color 0..5] from a generated header; a pre-2026-11-15
    header (one table, HOLE_<h>) counts as the same table for every color."""
    def parse(pattern):
        out = {}
        for m in re.finditer(pattern, text, re.S):
            key = tuple(int(g) for g in m.groups()[:-1])
            out[key] = [tuple(int(x) for x in p) for p in re.findall(r"\{(\d+), (\d+), (\d+)\}", m.groups()[-1])]
        return out
    colored = parse(r"constexpr Point C(\d+)_HOLE_(\d+)\[\] = \{[^\n]*\n(.*?)\};")
    if colored:
        by_color = [{} for _ in COLORS]
        for (c, h), pts in colored.items():
            if c < len(COLORS):
                by_color[c][h] = pts
        return by_color
    legacy = {h: pts for (h,), pts in parse(r"constexpr Point HOLE_(\d+)\[\] = \{(.*?)\};").items()}
    return [dict(legacy) for _ in COLORS]


def describe_changes_all(old, new):
    out = []
    for c, zh in enumerate(COLORS):
        out += [f"  [{zh}]" + line[1:] for line in describe_changes(old[c], new[c])]
    return out


def describe_changes(old, new):
    out = []
    for h in HOLES:
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
            + axis(510000004, 510000003, "r", "仰角 (µs)", 500, 2461, crosses="max", grid=False)
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


def write_xlsx(path, by_color):
    """The whole workbook from [points of color 0..5]: one data sheet per
    color (each with its three charts), 說明, and the hidden helper."""
    def cell(ref, value, style=0):
        s = f' s="{style}"' if style else ""
        if isinstance(value, str):
            return f'<c r="{ref}" t="inlineStr"{s}><is><t>{escape(value)}</t></is></c>'
        return f'<c r="{ref}"{s}><v>{value}</v></c>'

    def data_rows(points):
        rows = [[COLS["hole"], COLS["dist"], COLS["elev"], COLS["fly"], COLS["unit"]]]
        for h in HOLES:
            for cm, permille, us in points.get(h, []):
                rows.append([h, round(cm / 100, 2), us, round(permille / 10, 1), "%"])
        out = []
        for i, r in enumerate(rows, 1):
            cs = "".join(cell(cell_name(i, j), v, 1 if i == 1 else 0) for j, v in enumerate(r, 1))
            out.append(f'<row r="{i}">{cs}</row>')
        return out
    # Excel's own check on 轉速, by its unit (no backslash inside the f-string
    # below: Python before 3.12 rejects that).
    FLY_RULE = 'OR(AND(E2="%",D2>=0,D2<=100),AND(E2="us",D2>=1000,D2<=2000,INT(D2)=D2))'
    dv = (
        '<dataValidations count="5">'
        '<dataValidation type="whole" allowBlank="1" showErrorMessage="1" errorTitle="洞口" '
        f'error="洞口要是 1–12 的整數（10–12 是加分框左中右）" sqref="A2:A{MAX_ROWS}"><formula1>1</formula1><formula2>12</formula2></dataValidation>'
        '<dataValidation type="decimal" allowBlank="1" showErrorMessage="1" errorTitle="距離" '
        f'error="距離 0.5–15 m" sqref="B2:B{MAX_ROWS}"><formula1>0.5</formula1><formula2>15</formula2></dataValidation>'
        '<dataValidation type="whole" allowBlank="1" showErrorMessage="1" errorTitle="仰角" '
        f'error="仰角 500–2461 的整數 µs" sqref="C2:C{MAX_ROWS}"><formula1>500</formula1><formula2>2461</formula2></dataValidation>'
        '<dataValidation type="custom" allowBlank="1" showErrorMessage="1" errorTitle="轉速" '
        'error="單位 % 時 0–100；單位 us 時 1000–2000 的整數" '
        f'sqref="D2:D{MAX_ROWS}"><formula1>{escape(FLY_RULE)}</formula1></dataValidation>'
        '<dataValidation type="list" allowBlank="1" showErrorMessage="1" errorTitle="轉速單位" '
        f'error="只能選 % 或 us" sqref="E2:E{MAX_ROWS}"><formula1>"%,us"</formula1></dataValidation>'
        '</dataValidations>')
    def data_sheet(points, selected):
        tab = ' tabSelected="1"' if selected else ""
        return (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
            f'<sheetViews><sheetView{tab} workbookViewId="0"><pane ySplit="1" topLeftCell="A2" '
            'activePane="bottomLeft" state="frozen"/></sheetView></sheetViews>'
            '<cols><col min="1" max="5" width="11" customWidth="1"/></cols>'
            f'<sheetData>{"".join(data_rows(points))}</sheetData>{dv}<drawing r:id="rId1"/></worksheet>')

    # Sheet 3 (hidden): for each hole, its points sorted by distance --
    # distance / elevation / flywheel -- as formulas over sheet 1, so the
    # charts follow every edit, added row and deleted row by themselves.
    # One block of len(CHART_HOLES) * 3 helper columns per color (slot =
    # color * len(CHART_HOLES) + height), each over that color's sheet.
    slots = [(c, h) for c in range(len(COLORS)) for h, _ in CHART_HOLES]
    helper_rows = []
    head = []
    for slot, (c, h) in enumerate(slots):
        base = 1 + slot * 3
        for k, label in enumerate((f"{COLORS[c]} 洞口{h} 距離", f"{COLORS[c]} 洞口{h} 仰角", f"{COLORS[c]} 洞口{h} 轉速")):
            head.append(cell(cell_name(1, base + k), label, 1))
    helper_rows.append(f'<row r="1">{"".join(head)}</row>')
    for n in range(1, SORTED_ROWS + 1):
        r = n + 1
        cs = []
        for slot, (c, h) in enumerate(slots):
            t = _q(SHEETS[c])
            hole_rng = f"{t}!$A$2:$A${MAX_ROWS}"
            dist_rng = f"{t}!$B$2:$B${MAX_ROWS}"
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

    try:
        fly_side = "1000–1500" if flywheel_inverted() else "1500–2000"
    except TableError:
        fly_side = "見 TuningConstants.h"
    help_lines = [
        "射擊表怎麼用",
        "",
        "0. 每種球色一頁（2026-11-15）：射擊表-黃、紫、藍、綠、紅、橘。Dashboard 的 Match Setup 選球色，Mega 就用那一頁的表；某一頁沒有任何資料時用黃色那頁。",
        "1. 在各色的「射擊表-…」工作表改數字、加列、刪列，存檔（保持 .xlsx）。每頁旁邊三張射擊曲線圖（下、中、上排，用中間那一行的洞口 4、5、6）會自動跟著更新。",
        "2. 在專案資料夾執行：python3 tools/shot_table/apply.py --upload",
        "   會檢查格式、列出改了哪些值、產生 src/Constants/ShotTableData.h，再燒進 Mega。",
        "   只想檢查：python3 tools/shot_table/apply.py --check",
        "3. Dashboard 按 S 記錄的點：在 Dashboard 按「下載 CSV」，關掉 Excel。整份重測：python3 tools/shot_table/apply.py --replace --upload（那個球色的頁整頁換成 CSV）；局部修正：python3 tools/shot_table/apply.py --adjust --upload（每個點蓋掉同洞口距離最接近、0.3 m 以內的舊點，沒有就新增）。依 CSV 的「球色」欄放進那一色的頁；舊的 CSV 沒有球色欄時要加 --color 黃 之類。",
        "",
        "格式（嚴格）",
        "- 第 1 列是欄位名稱。「洞口」「距離(m)」「仰角(us)」「轉速」「轉速單位」五個必須各出現一次、名稱一字不差。",
        "- 若插入其他欄位，新欄位一定要有名稱；它們不會被讀取。",
        "- 洞口：Dashboard 的編號 1–9（1 左下、2 左中、3 左上 … 9 右上）；10、11、12 是最上面的加分框左、中、右（Dashboard 按 Q、W、E 選）。",
        "- 距離：鎖定目標時 Dashboard 顯示的相機距離（不是捲尺），0.5–15 m，最多 2 位小數。",
        "- 仰角：Dashboard 的仰角脈寬，500–2461 的整數。",
        "- 轉速單位：用下拉選單選「%」或「us」，每一列可以不同。",
        f"- 轉速：單位 % 時 0–100、最多 1 位小數；單位 us 時填 Dashboard「Falcon PWM」的脈寬整數，必須在正轉那一側（依 TuningConstants 的 Flywheel::INVERTED：false 是 1500–2000、true 是 1000–1500；產生這份 Excel 時是 {fly_side}），會自動換算成 %。",
        "- 列的順序不重要；空白列會略過；同一個洞口的同一個距離不能重複。",
        f"- 資料請放在第 2–{MAX_ROWS} 列；每個洞口最多 {SORTED_ROWS} 個點會畫進曲線圖。",
        "- 任何一格不符合，就不會產生檔案、也不會燒錄，畫面會指出是哪一格。",
        "",
        "沒有資料的洞",
        "- 會借用同一排中間洞的值：1、7 用 4，2、8 用 5，3、9 用 6，加分框 10、12 用 11（W）。",
        "- 中間洞（4、5、6、11）自己沒有資料時，那一排自動模式都不會發射。",
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

    n_colors = len(COLORS)
    n_charts = len(CHART_HOLES)
    ct = "application/vnd.openxmlformats-officedocument"
    rel = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
    xml_head = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
    # Sheets: 1..6 the colors, 7 說明, 8 the hidden helper.
    help_sheet = n_colors + 1
    helper_sheet = n_colors + 2
    files = {
        "[Content_Types].xml":
            xml_head +
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
            '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
            '<Default Extension="xml" ContentType="application/xml"/>'
            f'<Override PartName="/xl/workbook.xml" ContentType="{ct}.spreadsheetml.sheet.main+xml"/>'
            + "".join(f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType="{ct}.spreadsheetml.worksheet+xml"/>'
                      for i in range(1, helper_sheet + 1))
            + f'<Override PartName="/xl/styles.xml" ContentType="{ct}.spreadsheetml.styles+xml"/>'
            + "".join(f'<Override PartName="/xl/drawings/drawing{c + 1}.xml" ContentType="{ct}.drawing+xml"/>'
                      for c in range(n_colors))
            + "".join(f'<Override PartName="/xl/charts/chart{i + 1}.xml" ContentType="{ct}.drawingml.chart+xml"/>'
                      for i in range(n_colors * n_charts)) +
            '</Types>',
        "_rels/.rels":
            xml_head +
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
            '</Relationships>',
        "xl/workbook.xml":
            xml_head +
            '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
            f'xmlns:r="{rel}"><sheets>'
            + "".join(f'<sheet name="{escape(SHEETS[c])}" sheetId="{c + 1}" r:id="rId{c + 1}"/>' for c in range(n_colors))
            + f'<sheet name="說明" sheetId="{help_sheet}" r:id="rId{help_sheet}"/>'
            f'<sheet name="{HELPER}" sheetId="{helper_sheet}" state="hidden" r:id="rId{helper_sheet}"/></sheets>'
            '<calcPr calcId="191029" fullCalcOnLoad="1"/>'
            '</workbook>',
        "xl/_rels/workbook.xml.rels":
            xml_head +
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            + "".join(f'<Relationship Id="rId{i}" Type="{rel}/worksheet" Target="worksheets/sheet{i}.xml"/>'
                      for i in range(1, helper_sheet + 1))
            + f'<Relationship Id="rId{helper_sheet + 1}" Type="{rel}/styles" Target="styles.xml"/>'
            '</Relationships>',
        "xl/styles.xml":
            xml_head +
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
        f"xl/worksheets/sheet{help_sheet}.xml": sheet2,
        f"xl/worksheets/sheet{helper_sheet}.xml": sheet3,
    }
    for c in range(n_colors):
        files[f"xl/worksheets/sheet{c + 1}.xml"] = data_sheet(by_color[c], c == 0)
        files[f"xl/worksheets/_rels/sheet{c + 1}.xml.rels"] = (
            xml_head + '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            f'<Relationship Id="rId1" Type="{rel}/drawing" Target="../drawings/drawing{c + 1}.xml"/></Relationships>')
        files[f"xl/drawings/drawing{c + 1}.xml"] = (
            xml_head +
            '<xdr:wsDr xmlns:xdr="http://schemas.openxmlformats.org/drawingml/2006/spreadsheetDrawing" '
            'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
            f'xmlns:r="{rel}">'
            + "".join(_anchor(i + 1, 5, i * 20, 15, i * 20 + 19, f"rId{i + 1}", f"射擊曲線{i + 1}")
                      for i in range(n_charts))
            + '</xdr:wsDr>')
        files[f"xl/drawings/_rels/drawing{c + 1}.xml.rels"] = (
            xml_head + '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            + "".join(f'<Relationship Id="rId{i + 1}" Type="{rel}/chart" Target="../charts/chart{c * n_charts + i + 1}.xml"/>'
                      for i in range(n_charts))
            + '</Relationships>')
        for i, (_, title) in enumerate(CHART_HOLES):
            slot = c * n_charts + i
            files[f"xl/charts/chart{slot + 1}.xml"] = _height_chart_xml(slot, f"{COLORS[c]}球 {title}")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for name, body in files.items():
            z.writestr(name, body)


# ----------------------------------------------------------------- import

# The Dashboard's shot log (S key, 2026-10-26) -> rows of the Excel. Only
# the sheet's own XML is edited (rows added / cells replaced as text), so
# everything else in the workbook -- the charts, the hidden helper sheet,
# the dropdowns, extra columns, Excel's own formatting -- stays byte for
# byte as it was. A real XML round trip (ElementTree) would rename Excel's
# namespace prefixes and Excel would then call the file corrupt.

DOWNLOADS = os.path.expanduser(os.path.join("~", "Downloads"))
ROW_RE = re.compile(r'<row\b[^>]*?\br="(\d+)"[^>]*?(?:/>|>.*?</row>)', re.S)
CELL_RE = re.compile(r'<c\b[^>]*?\br="([A-Z]+)(\d+)"[^>]*?(?:/>|>.*?</c>)', re.S)


def newest_shot_log():
    logs = glob.glob(os.path.join(DOWNLOADS, "shot_log_*.csv"))
    if not logs:
        raise TableError(f"{DOWNLOADS} 裡沒有 shot_log_*.csv（先在 Dashboard 按「下載 CSV」，或把 CSV 路徑寫在 --import 後面）")
    return max(logs, key=os.path.getmtime)


def read_csv(path):
    """{(row, col): text}, 1-based like a sheet, from the Dashboard's CSV."""
    if not os.path.exists(path):
        raise TableError(f"找不到 CSV：{path}")
    cells = {}
    with open(path, encoding="utf-8-sig", newline="") as f:
        for r, cols in enumerate(csv.reader(f), 1):
            for c, text in enumerate(cols, 1):
                text = text.strip()
                if text != "":
                    cells[(r, c)] = text
    return cells


def fmt_meters(cm):
    text = f"{cm // 100}.{cm % 100:02d}".rstrip("0").rstrip(".")
    return text


def fmt_percent(permille):
    return f"{permille // 10}" if permille % 10 == 0 else f"{permille // 10}.{permille % 10}"


def _cell_xml(ref, value, old=None):
    """A cell with this value, keeping an old cell's style (s=)."""
    m = re.search(r'\ss="(\d+)"', old.split(">", 1)[0]) if old else None
    style = f' s="{m.group(1)}"' if m else ""
    if isinstance(value, str):
        return f'<c r="{ref}"{style} t="inlineStr"><is><t>{escape(value)}</t></is></c>'
    return f'<c r="{ref}"{style}><v>{value}</v></c>'


def _edit_row(row_xml, row, values):
    """values {col: value} into one <row> element (self-closing or not)."""
    head = re.match(r"<row\b[^>]*?(/?)>", row_xml)
    open_tag = head.group(0)
    inner = "" if head.group(1) else row_xml[len(open_tag):-len("</row>")]
    if head.group(1):
        open_tag = open_tag[:-2].rstrip() + ">"
    # spans= is only a hint for the cells' range; drop it rather than keep a
    # wrong one.
    open_tag = re.sub(r'\sspans="[^"]*"', "", open_tag)
    cells = {col_index(m.group(1)): m.group(0) for m in CELL_RE.finditer(inner)}
    rest = CELL_RE.sub("", inner)  # e.g. an extLst: kept after the cells
    for col, value in values.items():
        ref = cell_name(row, col)
        cells[col] = _cell_xml(ref, value, cells.get(col))
    return open_tag + "".join(cells[c] for c in sorted(cells)) + rest + "</row>"


def set_cells(sheet_xml, edits):
    """edits {row: {col: value}} -> the sheet XML with those cells set."""
    m = re.search(r"<sheetData\s*/>|<sheetData\b[^>]*>(.*?)</sheetData>", sheet_xml, re.S)
    if m is None:
        raise TableError("讀不懂「射擊表」的內容（沒有 sheetData）")
    inner = m.group(1) or ""
    rows = {int(r.group(1)): r.group(0) for r in ROW_RE.finditer(inner)}
    if ROW_RE.sub("", inner).strip():
        raise TableError("「射擊表」的列裡有看不懂的內容，為了安全不寫入（請用 Excel 另存一次再試）")
    for row, values in edits.items():
        rows[row] = _edit_row(rows.get(row, f'<row r="{row}"/>'), row, values)
    data = "<sheetData>" + "".join(rows[r] for r in sorted(rows)) + "</sheetData>"
    out = sheet_xml[:m.start()] + data + sheet_xml[m.end():]
    # Keep the sheet's declared used range covering the new rows.
    last_row = max(rows)
    last_col = max([c for v in edits.values() for c in v] + [1])

    def grow(d):
        end_col = d.group(3) or d.group(1)
        end_row = int(d.group(4) or d.group(2))
        col = cell_name(1, max(col_index(end_col), last_col))[:-1]
        return f'<dimension ref="{d.group(1)}{d.group(2)}:{col}{max(end_row, last_row)}"'
    return re.sub(r'<dimension ref="([A-Z]+)(\d+)(?::([A-Z]+)(\d+))?"', grow, out, count=1)


def full_calc_on_load(wb_xml):
    """Excel recalculates every formula when it opens the file, so the
    hidden chart sheet (and the charts) pick up the new rows."""
    m = re.search(r"<calcPr\b[^>]*?/?>", wb_xml)
    if m:
        tag = m.group(0)
        if "fullCalcOnLoad=" in tag:
            tag = re.sub(r'fullCalcOnLoad="[^"]*"', 'fullCalcOnLoad="1"', tag)
        else:
            tag = tag.replace("<calcPr", '<calcPr fullCalcOnLoad="1"', 1)
        return wb_xml[:m.start()] + tag + wb_xml[m.end():]
    # No calcPr: it goes after these, in the schema's order.
    pos = -1
    for end in ("</sheets>", "</functionGroups>", "<functionGroups/>",
                "</externalReferences>", "</definedNames>", "<definedNames/>"):
        i = wb_xml.rfind(end)
        if i >= 0:
            pos = max(pos, i + len(end))
    if pos < 0:
        return wb_xml
    return wb_xml[:pos] + '<calcPr fullCalcOnLoad="1"/>' + wb_xml[pos:]


def plan_import(cells, csv_cells, csv_name, near_cm=30):
    """What the CSV changes in the sheet: (edits, report lines, counts).
    Adjust (2026-11-16): a CSV point replaces the existing point of the same
    hole whose distance is nearest within near_cm (its distance too), each
    existing point at most once; none that close -> a new row."""
    if any(r > 1 for (r, c) in cells):
        try:
            parse_points(cells)  # the Excel itself must be valid before it is touched
        except TableError as e:
            raise TableError(f"Excel 本身有錯誤，先修正再匯入：\n{e}")
        found, old_rows = parse_rows(cells)
    else:
        # A color sheet with no rows yet (2026-11-15): only its headers.
        header = {v: c for (r, c), v in cells.items() if r == 1}
        missing = [name for name in COLS.values() if name not in header]
        if missing:
            raise TableError(f"Excel 那一頁第 1 列缺少欄位：{'、'.join(missing)}")
        found = {k: header[name] for k, name in COLS.items()}
        old_rows = []
    try:
        _, new_rows = parse_rows(csv_cells, "CSV ")
    except TableError as e:
        raise TableError(f"{csv_name} 有錯誤（格子位置是 CSV 的欄列）：\n{e}")
    # The same hole and distance twice in the log: the later press wins
    # (the Dashboard already keeps only the last one).
    latest = {}
    for row, h, cm, permille, elev in new_rows:
        latest.pop((h, cm), None)
        latest[(h, cm)] = (permille, elev)
    next_row = max(r for (r, c) in cells) + 1
    edits, lines = {}, []
    added = replaced = same = 0
    used = set()
    fmt = lambda permille, elev: f"{elev} us / {permille / 10:.1f}%"
    for (h, cm), (permille, elev) in latest.items():
        values = {found["elev"]: elev, found["fly"]: Decimal(fmt_percent(permille)), found["unit"]: "%"}
        # The nearest existing point of this hole within near_cm (an exact
        # distance is nearest of all).
        match = None
        for row, oh, ocm, operm, oelev in old_rows:
            if oh != h or row in used or abs(ocm - cm) > near_cm:
                continue
            if match is None or abs(ocm - cm) < abs(match[1] - cm):
                match = (row, ocm, operm, oelev)
        if match is not None:
            row, old_cm, old_permille, old_elev = match
            used.add(row)
            if (old_cm, old_permille, old_elev) == (cm, permille, elev):
                same += 1
                lines.append(f"  = 洞口 {h} {cm / 100:.2f} m：{fmt(permille, elev)}（第 {row} 列，已經一樣）")
                continue
            values[found["dist"]] = Decimal(fmt_meters(cm))
            replaced += 1
            moved = f"{old_cm / 100:.2f} m → {cm / 100:.2f} m，" if old_cm != cm else f"{cm / 100:.2f} m："
            lines.append(f"  * 洞口 {h} {moved}{fmt(old_permille, old_elev)} -> {fmt(permille, elev)}（取代第 {row} 列）")
        else:
            row = next_row
            next_row += 1
            values[found["hole"]] = h
            values[found["dist"]] = Decimal(fmt_meters(cm))
            added += 1
            lines.append(f"  + 洞口 {h} {cm / 100:.2f} m：{fmt(permille, elev)}（新增第 {row} 列）")
        edits[row] = values
    if next_row - 1 > MAX_ROWS:
        lines.append(f"  注意：資料超過第 {MAX_ROWS} 列，超過的列不會畫進曲線圖（射擊表照樣會用）")
    return edits, lines, (added, replaced, same)


def csv_by_color(csv_cells, csv_name, default_color):
    """{color index: CSV cells of that color's rows (header row kept)}. The
    Dashboard writes the color in a 球色 column (2026-11-15); an older CSV
    has none and needs --color."""
    header = {c: v for (r, c), v in csv_cells.items() if r == 1}
    color_cols = [c for c, v in header.items() if v == COLOR_COL]
    rows = sorted({r for (r, c) in csv_cells if r > 1})
    groups = {}
    errors = []
    for r in rows:
        if color_cols:
            text = csv_cells.get((r, color_cols[0]), "")
            name = text[:1]
            if name not in COLORS:
                errors.append(f"{csv_name} 第 {r} 列：球色要是 {'、'.join(COLORS)} 其中一個，現在是「{text}」")
                continue
            color = COLORS.index(name)
        elif default_color is not None:
            color = default_color
        else:
            raise TableError(f"{csv_name} 沒有「{COLOR_COL}」欄（舊版 Dashboard 的紀錄）：加上 --color 黃（或紫藍綠紅橘）指定放進哪一色")
        groups.setdefault(color, []).append(r)
    if errors:
        raise TableError("\n".join(errors))
    out = {}
    for color, rs in groups.items():
        sub_cells = {}
        for new_r, r in enumerate([1] + rs, 1):
            for (rr, c), v in csv_cells.items():
                if rr == r and c not in color_cols:
                    sub_cells[(new_r, c)] = v
        out[color] = sub_cells
    return out


def import_csv(xlsx, csv_path, dry_run, default_color=None, near_cm=30):
    lock = os.path.join(os.path.dirname(xlsx), "~$" + os.path.basename(xlsx))
    if os.path.exists(lock) and not dry_run:
        raise TableError(f"Excel 正開著 {os.path.basename(xlsx)}（有 {os.path.basename(lock)}）。"
                         "請先存檔並關閉 Excel 再匯入；如果 Excel 已經關了，刪掉那個 ~$ 檔再試")
    csv_name = os.path.basename(csv_path)
    groups = csv_by_color(read_csv(csv_path), csv_name, default_color)
    plans = {}
    for color in sorted(groups):
        cells = read_sheet(xlsx, SHEETS[color])
        if not any(r > 1 for (r, c) in cells):
            # An empty color sheet: plan_import checks the sheet first, and
            # an empty one fails that -- treat it as having no rows yet.
            cells = {k: v for k, v in cells.items() if k[0] == 1}
        edits, lines, (added, replaced, same) = plan_import(cells, groups[color], csv_name, near_cm)
        print(f"匯入 {csv_path} -> {SHEETS[color]}：新增 {added}、取代 {replaced}、相同 {same}")
        for line in lines:
            print(line)
        if edits:
            merged = dict(cells)
            for row, values in edits.items():
                for col, value in values.items():
                    merged[(row, col)] = str(value)
            parse_points(merged)  # the merged sheet must still pass every check
            plans[color] = (edits, merged)
    if dry_run or not plans:
        return False

    with zipfile.ZipFile(xlsx) as z:
        infos = z.infolist()
        parts = {i.filename: z.read(i.filename) for i in infos}
        targets = {color: sheet_part(z, SHEETS[color]) for color in plans}
    for color, (edits, _) in plans.items():
        parts[targets[color]] = set_cells(parts[targets[color]].decode("utf-8"), edits).encode("utf-8")
    parts["xl/workbook.xml"] = full_calc_on_load(parts["xl/workbook.xml"].decode("utf-8")).encode("utf-8")
    # A copy of the file as it was, next to it (tools/shot_table/backup/,
    # git-ignored).
    backup_dir = os.path.join(os.path.dirname(os.path.abspath(xlsx)), "backup")
    os.makedirs(backup_dir, exist_ok=True)
    backup = os.path.join(backup_dir, time.strftime("shot_table_%Y%m%d_%H%M%S.xlsx"))
    shutil.copy2(xlsx, backup)
    tmp = xlsx + ".tmp"
    with zipfile.ZipFile(tmp, "w") as out:
        for info in infos:
            out.writestr(info, parts[info.filename], compress_type=info.compress_type)
    # Re-read what was written: the same checks, and the new rows really there.
    for color, (_, merged) in plans.items():
        if parse_points(read_sheet(tmp, SHEETS[color])) != parse_points(merged):
            os.remove(tmp)
            raise TableError("寫出的 Excel 讀回來不一致，沒有覆蓋原檔")
    try:
        os.replace(tmp, xlsx)
    except PermissionError:
        os.remove(tmp)
        raise TableError(f"無法覆蓋 {xlsx}（Excel 是不是還開著？）")
    print(f"已寫入 {xlsx}（原檔備份：{backup}）")
    return True


def replace_csv(xlsx, csv_path, dry_run, default_color=None, all_colors=False):
    """REPLACE (2026-11-16): the CSV's color sheet(s) -- or all six with
    all_colors -- become exactly the CSV's points. The workbook is rebuilt
    (write_xlsx) with the other colors as they were."""
    lock = os.path.join(os.path.dirname(xlsx), "~$" + os.path.basename(xlsx))
    if os.path.exists(lock) and not dry_run:
        raise TableError(f"Excel 正開著 {os.path.basename(xlsx)}（有 {os.path.basename(lock)}）。"
                         "請先存檔並關閉 Excel；如果 Excel 已經關了，刪掉那個 ~$ 檔再試")
    csv_name = os.path.basename(csv_path)
    groups = csv_by_color(read_csv(csv_path), csv_name, default_color)
    by_color = read_all(xlsx)
    new = [dict(p) for p in by_color]
    for color, cells in groups.items():
        try:
            points = parse_points(cells)
        except TableError as e:
            raise TableError(f"{csv_name}（{COLORS[color]}）有錯誤（格子位置是 CSV 的欄列）：\n{e}")
        targets = range(len(COLORS)) if all_colors else [color]
        for c in targets:
            new[c] = dict(points)
            old_n = sum(len(v) for v in by_color[c].values())
            new_n = sum(len(v) for v in points.values())
            print(f"整份換新 {SHEETS[c]}：舊的 {old_n} 個點全部刪掉，換成 {csv_name} 的 {new_n} 個點"
                  f"（" + "、".join(f"洞口 {h} {len(points[h])} 點" for h in sorted(points)) + "）")
    if all_colors and len(groups) > 1:
        raise TableError("--all-colors 只能用在一種球色的 CSV（這份有好幾種球色）")
    if dry_run:
        return False
    backup_dir = os.path.join(os.path.dirname(os.path.abspath(xlsx)), "backup")
    os.makedirs(backup_dir, exist_ok=True)
    backup = os.path.join(backup_dir, time.strftime("shot_table_%Y%m%d_%H%M%S.xlsx"))
    shutil.copy2(xlsx, backup)
    tmp = xlsx + ".tmp"
    write_xlsx(tmp, new)
    if read_all(tmp) != new:
        os.remove(tmp)
        raise TableError("寫出的 Excel 讀回來不一致，沒有覆蓋原檔")
    try:
        os.replace(tmp, xlsx)
    except PermissionError:
        os.remove(tmp)
        raise TableError(f"無法覆蓋 {xlsx}（Excel 是不是還開著？）")
    print(f"已寫入 {xlsx}（原檔備份：{backup}）")
    return True


def read_all(xlsx):
    """[points of color 0..5]; an empty color sheet (header only) = {}."""
    by_color = []
    errors = []
    for sheet in SHEETS:
        try:
            cells = read_sheet(xlsx, sheet)
        except TableError as e:
            if "沒有名為" in str(e):
                legacy = ""
                try:
                    read_sheet(xlsx, SHEET)
                    legacy = "（這是 2026-11-15 以前只有一頁「射擊表」的舊格式，請更新 Excel：git pull）"
                except TableError:
                    pass
                raise TableError(f"Excel 裡沒有「{sheet}」工作表{legacy}")
            raise
        if not any(r > 1 for (r, c) in cells):
            by_color.append({})
            continue
        try:
            by_color.append(parse_points(cells))
        except TableError as e:
            errors.append(f"「{sheet}」：\n{e}")
    if errors:
        raise TableError("\n".join(errors))
    return by_color


# ----------------------------------------------------------------- main

def find_pio():
    """The PlatformIO command as a list, or None: pio on PATH, the VS Code
    extension's own copy (Mac / Windows), or PlatformIO installed with pip
    into this Python (`py -m pip install platformio` on Windows, run as
    `python -m platformio`)."""
    for cand in (shutil.which("pio"), shutil.which("platformio"),
                 os.path.expanduser("~/.platformio/penv/bin/pio"),
                 os.path.expanduser("~/.platformio/penv/Scripts/pio.exe")):
        if cand and os.path.exists(cand):
            return [cand]
    import importlib.util
    if importlib.util.find_spec("platformio") is not None:
        return [sys.executable, "-m", "platformio"]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xlsx", default=XLSX)
    ap.add_argument("--check", action="store_true", help="check only, write nothing")
    ap.add_argument("--upload", action="store_true", help="also build and upload the Mega")
    ap.add_argument("--env", default="megaatmega2560")
    ap.add_argument("--export", action="store_true", help="rebuild the Excel from the current header")
    ap.add_argument("--import", dest="import_csv", nargs="?", const="", metavar="CSV",
                    help="put the Dashboard's shot log CSV into the Excel first "
                         "(no CSV = the newest shot_log_*.csv in ~/Downloads)")
    ap.add_argument("--color", choices=COLORS, default=None,
                    help="the color sheet for a CSV without a 球色 column")
    ap.add_argument("--replace", dest="replace_csv", nargs="?", const="", metavar="CSV",
                    help="REPLACE: the CSV's color sheet(s) become exactly the CSV")
    ap.add_argument("--adjust", dest="adjust_csv", nargs="?", const="", metavar="CSV",
                    help="ADJUST: each CSV point overwrites the nearest point within --near m")
    ap.add_argument("--near", type=float, default=0.3,
                    help="with --adjust / --import: meters within which a point counts as the same (0.3)")
    ap.add_argument("--all-colors", action="store_true",
                    help="with --replace: the same points into all six color sheets")
    args = ap.parse_args()
    # --import is --adjust (2026-11-16).
    if args.import_csv is not None and args.adjust_csv is None:
        args.adjust_csv = args.import_csv
    modes = [m for m in (args.export, args.replace_csv is not None, args.adjust_csv is not None) if m]
    if len(modes) > 1:
        sys.exit("--export、--replace、--adjust（--import）一次只能用一個")
    if args.all_colors and args.replace_csv is None:
        sys.exit("--all-colors 只能跟 --replace 一起用")

    old_text = open(HEADER, encoding="utf-8").read() if os.path.exists(HEADER) else ""
    old = parse_header(old_text)

    if args.export:
        if os.path.exists(args.xlsx):
            sys.exit(f"{args.xlsx} 已存在；要重建請先自己移走它（不會覆蓋）")
        write_xlsx(args.xlsx, old)
        print(f"已從 {os.path.relpath(HEADER, ROOT)} 產生 {os.path.relpath(args.xlsx, ROOT)}")
        return

    if args.replace_csv is not None or args.adjust_csv is not None:
        default_color = COLORS.index(args.color) if args.color else None
        try:
            if args.replace_csv is not None:
                csv_path = args.replace_csv or newest_shot_log()
                replace_csv(args.xlsx, csv_path, dry_run=args.check,
                            default_color=default_color, all_colors=args.all_colors)
            else:
                csv_path = args.adjust_csv or newest_shot_log()
                import_csv(args.xlsx, csv_path, dry_run=args.check,
                           default_color=default_color, near_cm=round(args.near * 100))
        except TableError as e:
            print("匯入失敗，Excel 沒有改動、也沒有燒錄：\n" + str(e), file=sys.stderr)
            sys.exit(1)
        if args.check:
            return
        print()

    try:
        by_color = read_all(args.xlsx)
    except TableError as e:
        print("格式錯誤，沒有寫入任何檔案、也沒有燒錄：\n" + str(e), file=sys.stderr)
        sys.exit(1)

    for c, points in enumerate(by_color):
        if not points:
            print(f"[{COLORS[c]}] 沒有資料：用黃色的表")
            continue
        total = sum(len(p) for p in points.values())
        print(f"[{COLORS[c]}] {total} 個點：" + "，".join(f"洞口 {h} {len(points[h])} 點" for h in sorted(points)))
        for h in [h for h in HOLES if h not in points]:
            src = FALLBACK.get(h)
            if src is None:
                note = "這一排的中間洞，沒有資料 -> 自動模式不會對它發射"
            else:
                note = f"借用洞口 {src}" if src in points else "同排中間洞也沒有 -> 自動模式不會對它發射"
            print(f"  洞口 {h}{'（' + BONUS[h] + '）' if h in BONUS else ''} 沒有資料：{note}")
    changes = describe_changes_all(old, by_color)
    print("改動：" if changes else "跟目前的射擊表相同，沒有改動。")
    for line in changes:
        print(line)

    if args.check:
        return
    new_text = render_header(by_color)
    if new_text != old_text:
        with open(HEADER, "w", encoding="utf-8", newline="\n") as f:
            f.write(new_text)
        print(f"已寫入 {os.path.relpath(HEADER, ROOT)}")
    if args.upload:
        pio = find_pio()
        if pio is None:
            sys.exit("找不到 PlatformIO（pio）。安裝：python3 -m pip install -U platformio"
                     "（Windows：py -m pip install -U platformio），或裝 VS Code 的 PlatformIO IDE。")
        print(f"編譯並燒錄 Mega（{args.env}）…")
        sys.exit(subprocess.call(pio + ["run", "-e", args.env, "-t", "upload"], cwd=ROOT))


if __name__ == "__main__":
    main()
