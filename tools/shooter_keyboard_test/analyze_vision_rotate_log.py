#!/usr/bin/env python3
"""Offline settling-time / max-overshoot / dt-distribution analysis for a
captured mega_vision_rotate_test $VROTATE,4/5/6,... log.

Why this exists (2026-10-10, Codex-reviewed PIDF stabilization pass, see
docs/codex-handoff.md): the Mega has limited RAM/flash and a real-time
control loop to run -- computing settling time/overshoot statistics on the
firmware itself would spend control-loop budget on something that's
fundamentally an offline analysis task. The v4 telemetry format already
carries everything needed (tx, dtMs, P/D components, pre/post-saturation
output) for this script to compute those numbers from a plain captured log
(copy-pasted from a terminal, Serial Monitor, or the browser page's own
console), with no live connection to any device.

Usage:
    python3 analyze_vision_rotate_log.py <log_file> [--settle-threshold-px N]
        [--settle-frames N]

<log_file> is a plain text file containing captured serial output; only
lines matching "$VROTATE,4/5/6,..." are used, everything else (including the
one-time "$VROTATE_GAINS,..." boot banner, which is captured separately
and shown in the report so the gains that actually produced this log are
traceable) is ignored.

Deadband was retired from the firmware 2026-10-10 (Jeremy's explicit call,
overriding Codex's original "don't loosen the deadband" note), so there is
no fixed hardware px boundary to call "settled" anymore -- --settle-threshold-px
is this script's own adjustable stand-in for "close enough to call it
centered", not a firmware constant. Tune it to match what actually matters
for the mechanism; there is no single correct default.
"""

import argparse
import re
import statistics
import sys

# Field layouts after "$VROTATE,<version>,". v4/v5 are older logs; v6
# (2026-10-14) dropped the D43/D42 limit and no-limits fields when the limit
# switches were removed. A line whose field count doesn't match its version
# is skipped.
_V4 = ["enabled", "pulse", "left_limit", "right_limit", "no_limits", "reason",
       "tx", "ty", "distance", "target_id", "valid", "ms_since_vision",
       "dt_ms", "p_term_us", "d_term_us", "output_raw_us", "saturated"]
LAYOUTS = {
    "4": _V4,
    "5": _V4 + ["i_term_us"],
    "6": ["enabled", "pulse", "reason", "tx", "ty", "distance", "target_id",
          "valid", "ms_since_vision", "dt_ms", "p_term_us", "d_term_us",
          "output_raw_us", "saturated", "i_term_us"],
}
INT_FIELDS = {"pulse", "tx", "target_id", "ms_since_vision", "dt_ms",
              "p_term_us", "d_term_us", "output_raw_us", "i_term_us"}
GAINS_RE = re.compile(r"^\$VROTATE_GAINS,(.+)\s*$")


def parse_sample(line):
    parts = line.split(",")
    if len(parts) < 2 or parts[0] != "$VROTATE":
        return None
    layout = LAYOUTS.get(parts[1])
    fields = parts[2:]
    if layout is None or len(fields) != len(layout):
        return None
    raw = dict(zip(layout, fields))
    try:
        sample = {name: int(raw[name]) for name in INT_FIELDS if name in raw}
    except ValueError:
        return None
    sample.setdefault("i_term_us", 0)
    sample["enabled"] = raw["enabled"] == "1"
    sample["valid"] = raw["valid"] == "1"
    sample["saturated"] = raw["saturated"] == "1"
    return sample


def parse_log(path):
    samples = []
    gains_line = None
    total_lines = 0
    matched_lines = 0
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            total_lines += 1
            line = raw_line.strip()
            gains_match = GAINS_RE.match(line)
            if gains_match:
                gains_line = gains_match.group(1)
                continue
            sample = parse_sample(line)
            if sample is None:
                continue
            matched_lines += 1
            samples.append(sample)
    return samples, gains_line, total_lines, matched_lines


def compute_elapsed_ms(samples):
    """Reconstruct a running clock from dt_ms (dt_ms==0 means a same-tick
    burst/duplicate or a filter-reset sample -- no time advances for those,
    consistent with how the firmware itself treats them)."""
    elapsed = 0
    out = []
    for s in samples:
        elapsed += s["dt_ms"]
        out.append(elapsed)
    return out


def find_settling_time(samples, elapsed_ms, threshold_px, required_frames):
    """First elapsed_ms at which `required_frames` consecutive valid,
    enabled samples all have |tx| <= threshold_px. Returns None if the log
    never reaches that condition."""
    run_length = 0
    for i, s in enumerate(samples):
        if s["enabled"] and s["valid"] and abs(s["tx"]) <= threshold_px:
            run_length += 1
            if run_length >= required_frames:
                start_index = i - required_frames + 1
                return elapsed_ms[start_index]
        else:
            run_length = 0
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log_file")
    parser.add_argument(
        "--settle-threshold-px",
        type=int,
        default=2,
        help="max |tx| (pixels) to count as 'centered' for settling-time purposes (default: 2)",
    )
    parser.add_argument(
        "--settle-frames",
        type=int,
        default=10,
        help="consecutive centered samples required to declare settled (default: 10)",
    )
    args = parser.parse_args()

    samples, gains_line, total_lines, matched_lines = parse_log(args.log_file)

    print(f"Read {total_lines} lines, {matched_lines} matched $VROTATE,4/5/6,...")
    if gains_line:
        print(f"Gains banner from this log: {gains_line}")
    else:
        print("WARNING: no $VROTATE_GAINS banner found in this log -- the gains that")
        print("         produced this run cannot be confirmed from the log itself.")

    if not samples:
        print("No $VROTATE,4/5/6,... samples found -- nothing to analyze.")
        sys.exit(1)

    elapsed_ms = compute_elapsed_ms(samples)

    enabled_valid = [s for s in samples if s["enabled"] and s["valid"]]
    if not enabled_valid:
        print("No samples with enabled=1 and valid=1 -- never actually tracking a target.")
        sys.exit(0)

    abs_tx_values = [abs(s["tx"]) for s in enabled_valid]
    max_overshoot = max(abs_tx_values)
    max_overshoot_index = abs_tx_values.index(max_overshoot)
    max_overshoot_sample = enabled_valid[max_overshoot_index]

    settling_ms = find_settling_time(
        samples, elapsed_ms, args.settle_threshold_px, args.settle_frames
    )

    dt_values = [s["dt_ms"] for s in samples if s["dt_ms"] > 0]
    saturated_count = sum(1 for s in enabled_valid if s["saturated"])

    print()
    print(f"Samples: {len(samples)} total, {len(enabled_valid)} enabled+valid")
    print(f"Max |tx| (overshoot): {max_overshoot}px (tx={max_overshoot_sample['tx']}, "
          f"target_id={max_overshoot_sample['target_id']})")
    if settling_ms is None:
        print(
            f"Settling time: NEVER reached {args.settle_frames} consecutive samples "
            f"within |tx|<={args.settle_threshold_px}px in this log -- report this "
            f"honestly, don't assume it would have settled given more time."
        )
    else:
        print(
            f"Settling time: {settling_ms} ms (first run of "
            f"{args.settle_frames} consecutive samples within "
            f"|tx|<={args.settle_threshold_px}px)"
        )
    print(f"Saturated samples (hit +-output limit): {saturated_count}/{len(enabled_valid)}")

    if dt_values:
        print()
        print("Received-interval (dt) distribution, ms (this is actual received spacing,")
        print("not assumed from a nominal frame rate):")
        print(f"  min={min(dt_values)} median={statistics.median(dt_values):.1f} "
              f"max={max(dt_values)} n={len(dt_values)}")
        if len(dt_values) >= 20:
            sorted_dt = sorted(dt_values)
            p95_index = min(len(sorted_dt) - 1, int(len(sorted_dt) * 0.95))
            print(f"  p95={sorted_dt[p95_index]}")
    else:
        print("No samples with dt_ms>0 -- cannot report an interval distribution.")


if __name__ == "__main__":
    main()
