#!/usr/bin/env python3
"""
Static layout check for the 320x240 panel.

WHY THIS EXISTS

Nothing at compile time knows how big the screen is. A button at y=200 with
h=32 ends at 231 and is fine; the same button at y=220 silently overlaps the
tab bar and becomes unreachable. A 54-character string at x=8 runs off the
right edge and clips mid-word. Both compile perfectly and both were shipped
at least once during development.

This parses the sketch and checks:
  * every rect_t fits inside 320x240
  * no rect_t overlaps the tab bar, except the tab bar's own row
  * every literal draw_text / draw_text_12 call fits horizontally
  * procedure page bodies fit between the divider and the status line
  * list rows, the scroll column and row text do not collide

It is deliberately conservative about what it parses: only calls with literal
integer coordinates. Anything computed at runtime is skipped rather than
guessed at, so a pass here is a floor, not a ceiling.

    python3 test/layout_check.py
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
INO = os.path.join(HERE, "..", "Vulpecula", "Vulpecula.ino")

SCR_W, SCR_H = 320, 240
CW, CH = 6, 8            # 6x8 glyph cell
CW12, CH12 = 12, 16      # 12x16 glyph cell

problems = []
notes = []


def define(src, name, default=None):
    m = re.search(r"^#define\s+" + name + r"\s+(\d+)", src, re.M)
    if m:
        return int(m.group(1))
    if default is None:
        problems.append(f"#define {name} not found")
        return 0
    return default


def main():
    src = open(INO).read()

    tabbar_y = define(src, "TABBAR_Y")
    tab_h = define(src, "TAB_H")
    list_y0 = define(src, "LIST_Y0")
    row_h = define(src, "ROW_H")
    list_rows = define(src, "LIST_ROWS")
    scroll_x = define(src, "SCROLL_X")
    scroll_w = define(src, "SCROLL_W")
    row_chars = define(src, "ROW_CHARS")

    # ---- rect_t bounds and tab-bar overlap ----------------------------------
    rects = {}
    for m in re.finditer(
            r"static const rect_t (\w+)\s*=\s*\{\s*([A-Za-z0-9_+\- ]+),"
            r"\s*([A-Za-z0-9_+\- ]+),\s*([A-Za-z0-9_+\- ]+),\s*(\d+)\s*\}", src):
        name = m.group(1)
        vals = []
        ok = True
        for expr in m.groups()[1:]:
            expr = expr.strip()
            # resolve the handful of #define names used in these initialisers
            for dn in ("SCROLL_X", "SCROLL_W", "LIST_Y0", "ROW_H"):
                expr = re.sub(r"\b" + dn + r"\b", str(define(src, dn)), expr)
            try:
                vals.append(int(eval(expr, {"__builtins__": {}}, {})))
            except Exception:
                ok = False
                break
        if not ok:
            notes.append(f"{name}: non-literal geometry, skipped")
            continue
        x, y, w, h = vals
        rects[name] = (x, y, w, h)
        if x < 0 or y < 0 or x + w > SCR_W or y + h > SCR_H:
            problems.append(f"{name} at ({x},{y}) {w}x{h} leaves the screen "
                            f"(right={x+w}, bottom={y+h})")
        # Procedure pages have no tab bar, so their buttons may sit below it.
        if not name.startswith("BTN_INFO") and y + h > tabbar_y:
            problems.append(f"{name} bottom={y+h} overlaps the tab bar at "
                            f"y={tabbar_y}")

    # ---- literal text calls fit horizontally --------------------------------
    for pat, cw, ch, label in (
            (r'draw_text\(\s*(\d+)\s*,\s*(\d+)\s*,\s*"([^"]*)"', CW, CH, "6x8"),
            (r'draw_text_12\(\s*(\d+)\s*,\s*(\d+)\s*,\s*"([^"]*)"', CW12, CH12, "12x16")):
        for m in re.finditer(pat, src):
            x, y, t = int(m.group(1)), int(m.group(2)), m.group(3)
            right = x + len(t) * cw
            if right > SCR_W:
                problems.append(f'{label} text at x={x} is {right}px wide '
                                f'(max {SCR_W}): "{t[:44]}"')
            if y + ch > SCR_H:
                problems.append(f'{label} text at y={y} ends at {y+ch}: '
                                f'"{t[:30]}"')

    # ---- procedure page bodies fit above the status line --------------------
    stat_y = None
    m = re.search(r"field_init\(&F_INFO_STAT,\s*\d+,\s*(\d+)", src)
    if m:
        stat_y = int(m.group(1))
    body_y0, body_step = 28, 12
    m = re.search(r"draw_text\(6,\s*28 \+ i \* 12", src)
    if not m:
        notes.append("procedure body layout expression changed; check by hand")

    for name in ("INFO_BASELINE", "INFO_RSSICAL"):
        if ("static const infopage_t " + name) not in src:
            problems.append(f"{name} not found")
            continue
        blk = src[src.index("static const infopage_t " + name):]
        blk = blk[:blk.index("NULL")]
        lines = re.findall(r'^\s+"([^"]*)",\s*$', blk, re.M)
        if not lines:
            problems.append(f"{name}: no body lines parsed")
            continue
        title, body = lines[0], lines[1:]
        last = body_y0 + (len(body) - 1) * body_step + CH
        if stat_y is not None and last > stat_y:
            problems.append(f"{name}: {len(body)} body lines end at y={last}, "
                            f"colliding with the status line at y={stat_y}")
        for t in body:
            txt = t.lstrip("!>")
            right = 6 + len(txt) * CW
            if right > SCR_W:
                problems.append(f'{name}: line is {right}px wide: "{txt[:44]}"')
        notes.append(f"{name}: title \"{title}\", {len(body)} body lines, "
                     f"last row y={last}")

    # ---- list rows, scroll column, row text --------------------------------
    rows_bottom = list_y0 + list_rows * row_h - 4
    if rows_bottom > tabbar_y:
        problems.append(f"list rows end at y={rows_bottom}, past the tab bar "
                        f"at y={tabbar_y}")
    if scroll_x + scroll_w > SCR_W:
        problems.append(f"scroll column ends at x={scroll_x+scroll_w}")
    row_text_right = 8 + row_chars * CW
    if row_text_right > scroll_x:
        problems.append(f"row text ends at x={row_text_right}, colliding with "
                        f"the scroll column at x={scroll_x}")
    notes.append(f"list: rows to y={rows_bottom}, row text to x={row_text_right}, "
                 f"scroll column from x={scroll_x}")

    # ---- report -------------------------------------------------------------
    for n in notes:
        print(f"  note: {n}")
    print()
    if problems:
        for p in problems:
            print(f"  FAIL: {p}")
        print(f"\nLAYOUT FAILED ({len(problems)} problem"
              f"{'' if len(problems) == 1 else 's'})\n")
        return 1
    print(f"LAYOUT OK - {len(rects)} rects and every literal string fit\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
