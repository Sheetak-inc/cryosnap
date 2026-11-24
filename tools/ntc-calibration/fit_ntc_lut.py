#!/usr/bin/env python3
"""
fit_ntc_lut.py - fit a nonlinear NTC calibration for CryoSnap and emit
the PROGMEM lookup table for NTC.h.

Companion to the "NTC Nonlinear Calibration" application note
(2026-09-01). The note is the
procedure; this script is the analysis step. Method:

  1. Align the CryoSnap Monitor log to the reference thermometer log
     (cross-correlation; the monitor filename timestamp is NOT the
     log start time, so never align by filename).
  2. Recover raw ADC counts from the reported temperature by
     inverting the conversion the firmware was running DURING the
     log (--fw linear or --fw lut), then resistance via the divider.
  3. Extract quasi-steady points (low slope on both channels, time
     since last setpoint change).
  4. Compute the sensor's datasheet-curve temperature from measured
     resistance, then fit a degree-2 correction on top of it,
     weighted bin means so dense clusters don't dominate.
  5. Emit: residual report, plot (if matplotlib present), and the
     C table for NTC.h at --step spacing.

Reuse for a DIFFERENT SENSOR: supply --rt-table with the new part's
published R-T curve as a CSV (temp_C,resistance_ohm). Reuse on FUTURE
FIRMWARE that already runs a LUT: pass --fw lut --lut-header NTC.h so
step 2 inverts the running table instead of the legacy line (any trim
the unit had saved must be identity during collection: run `defaults`
+ `save` first, or record the trim and pass --trim scale,offset).

Requires: numpy, pandas. matplotlib optional (plot skipped without it).

Example (the 2026-08-28 TK95F103W run):
  python fit_ntc_lut.py --monitor cryosnap_log_20260828_135606.csv \
      --ref DA-13-54.csv --fw linear
"""
import argparse, io, os, re, sys
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Built-in R-T curve: Amphenol Thermometrics material F, R25 = 10 kOhm.
# Source: AAS-913-318C Rev C, p.12 (Rt/R25 column x 10000).
# Override with --rt-table for any other sensor.
# ---------------------------------------------------------------------------
CURVE_F_10K = """temp_C,resistance_ohm
-50,686000\n-45,481600\n-40,342300\n-35,246200\n-30,179100\n-25,131700
-20,97820\n-15,73390\n-10,55580\n-5,42470\n0,32740\n5,25440\n10,19920
15,15720\n20,12500\n25,10000\n30,8056\n35,6530\n40,5326\n45,4369\n50,3604
55,2989\n60,2491\n65,2087\n70,1756\n75,1485\n80,1261\n85,1075\n90,920.9
95,791.6\n100,683.1\n105,591.6\n110,514.1"""

def frontend(args):
    """Forward model constants: divider + op-amp stage + ADC."""
    r23 = args.r2 * args.r3 / (args.r2 + args.r3)
    G    = (r23 + args.r4) / r23
    Voff = args.r4 / args.r3 * args.vdd
    return G, Voff

def raw_from_R(R, args):
    G, Voff = frontend(args)
    vin  = args.vdd * args.r1 / (R + args.r1)
    vout = vin * G - Voff
    return 1023.0 * vout / args.vref

def R_from_raw(raw, args):
    G, Voff = frontend(args)
    vout = raw * args.vref / 1023.0
    vin  = (vout + Voff) / G
    return args.r1 * (args.vdd / vin - 1.0)

# ---------------------------------------------------------------------------
def load_monitor(path):
    df = pd.read_csv(path)
    need = {"Time_s", "Setpoint_C", "Tcold_C"}
    if not need.issubset(df.columns):
        sys.exit(f"monitor CSV missing columns {need - set(df.columns)}")
    return df[df.Tcold_C.notna()].copy()

def load_reference(path, channel):
    """Calex Excelog export: 3 preamble rows, then Time / Sample No. /
    Channel N columns. Falls back to a plain time_s,temp_C CSV."""
    head = open(path, encoding="utf-8-sig").readline()
    if "time_s" in head.lower():
        df = pd.read_csv(path, encoding="utf-8-sig")
        return df.rename(columns=str.strip)[["time_s", "temp_C"]].rename(
            columns={"time_s": "tsec", "temp_C": "Tr"})
    df = pd.read_csv(path, skiprows=3, encoding="utf-8-sig")
    df.columns = [c.strip() for c in df.columns]
    df = df[df["Time"].notna()].copy()
    df["ts"] = pd.to_datetime(df["Time"], errors="coerce")
    df = df[df.ts.notna()]
    col = f"Channel {channel}"
    df["Tr"] = pd.to_numeric(df[col].astype(str).str.strip(), errors="coerce")
    df = df[df["Tr"].notna()]
    df["tsec"] = (df.ts - df.ts.iloc[0]).dt.total_seconds()
    return df[["tsec", "Tr"]]

def recover_raw(df, args):
    """Invert the conversion the firmware ran during collection."""
    T = df.Tcold_C.values
    if args.trim:
        s, o = args.trim
        T = (T - o) / s                     # undo saved trim first
    if args.fw == "linear":
        vin = (T - args.k0()) / args.k1()
        raw = None                          # not needed; go straight to R
        R = args.r1 * (args.vdd / vin - 1.0)
    else:                                   # fw == "lut": invert the table
        raw_t, t10 = parse_lut_header(args.lut_header)
        # table maps raw->T; invert by interpolating T->raw (monotonic)
        raw = np.interp(T, t10 / 10.0, raw_t)
        R = R_from_raw(raw, args)
    df = df.copy(); df["R"] = R
    return df

def parse_lut_header(path):
    txt = open(path).read()
    # Anchor on the PROGMEM declaration so mentions of the array names
    # in comments (e.g. NTC.h's own header block) cannot match.
    raw = re.search(r"NTC_LUT_RAW\[[^\]]*\]\s*PROGMEM\s*=\s*\{([^}]*)\}", txt)
    t10 = re.search(r"NTC_LUT_T10\[[^\]]*\]\s*PROGMEM\s*=\s*\{([^}]*)\}", txt)
    if not (raw and t10):
        sys.exit("could not find NTC_LUT_RAW / NTC_LUT_T10 in " + path)
    def f(m):
        body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)  # strip comments
        return np.array([int(x) for x in re.findall(r"-?\d+", body)], float)
    a, b = f(raw), f(t10)
    if len(a) != len(b) or len(a) < 2:
        sys.exit(f"LUT parse error: {len(a)} raw vs {len(b)} T10 entries")
    return a, b

def align(mon, ref, span=(-300, 500)):
    """Best lag (s) by cross-correlating reported temp vs reference."""
    grid = np.arange(0, min(mon.Time_s.max(), ref.tsec.max()), 1.0)
    mi = np.interp(grid, mon.Time_s, mon.Tcold_C)
    best = None
    for lag in np.arange(span[0], span[1], 1.0):
        ri = np.interp(grid + lag, ref.tsec, ref.Tr, left=np.nan, right=np.nan)
        m = ~np.isnan(ri)
        if m.sum() < min(1500, 0.5 * len(grid)):
            continue
        r = np.corrcoef(mi[m], ri[m])[0, 1]
        if best is None or r > best[1]:
            best = (lag, r)
    return best

def steady_points(mon, ref, lag, slope_lim, min_since_sp):
    d = mon[["Time_s", "Setpoint_C", "Tcold_C", "R"]].copy()
    d = d[(d.Time_s + lag >= ref.tsec.min()) & (d.Time_s + lag <= ref.tsec.max())]
    d["Tref"] = np.interp(d.Time_s + lag, ref.tsec, ref.Tr)
    d = d.sort_values("Time_s").reset_index(drop=True)
    t = d.Time_s.values
    def slope(y, win=30.0):
        out = np.full(len(t), np.nan)
        for i in range(len(t)):
            m = (t >= t[i] - win) & (t <= t[i] + win)
            if m.sum() >= 8:
                out[i] = np.polyfit(t[m], y[m], 1)[0] * 60.0   # C/min
        return out
    d["sl_m"] = slope(d.Tcold_C.values); d["sl_r"] = slope(d.Tref.values)
    spc = d.Setpoint_C.ffill().values
    chg = np.r_[True, spc[1:] != spc[:-1]]
    last = np.maximum.accumulate(np.where(chg, t, -1e9))
    d["since_sp"] = t - last
    return d[(d.sl_m.abs() < slope_lim) & (d.sl_r.abs() < slope_lim)
             & (d.since_sp > min_since_sp)].copy()

# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--monitor", required=True, help="CryoSnap Monitor CSV")
    p.add_argument("--ref", required=True, help="reference thermometer CSV")
    p.add_argument("--ref-channel", type=int, default=1)
    p.add_argument("--rt-table", help="sensor R-T CSV (temp_C,resistance_ohm); "
                                      "default = built-in Amphenol curve F 10k")
    p.add_argument("--fw", choices=["linear", "lut"], default="linear",
                   help="conversion the firmware ran DURING collection")
    p.add_argument("--fw-scale", type=float, default=0.1023)
    p.add_argument("--fw-offset", type=float, default=-27.6)
    p.add_argument("--lut-header", default="NTC.h",
                   help="NTC.h holding the running table (--fw lut)")
    p.add_argument("--trim", type=lambda s: tuple(map(float, s.split(","))),
                   help="scale,offset trim saved on the unit during collection")
    # front end (Rev B stock network)
    p.add_argument("--r1", type=float, default=10000.0)
    p.add_argument("--r2", type=float, default=4750.0)
    p.add_argument("--r3", type=float, default=4640.0)
    p.add_argument("--r4", type=float, default=68.1)
    p.add_argument("--vdd", type=float, default=3.3)
    p.add_argument("--vref", type=float, default=3.3)
    # extraction + fit + table
    p.add_argument("--slope-limit", type=float, default=1.0, help="C/min")
    p.add_argument("--min-since-sp", type=float, default=60.0, help="s")
    p.add_argument("--bin", type=float, default=2.0, help="C, fit bins")
    p.add_argument("--deg", type=int, default=2, help="correction degree")
    p.add_argument("--step", type=float, default=5.0, help="C, table spacing")
    p.add_argument("--table-range", type=lambda s: tuple(map(float, s.split(","))),
                   default=(-15.0, 110.0), help="C, table extent 'lo,hi'")
    p.add_argument("--out", default="ntc_lut_fit", help="output basename")
    args = p.parse_args()
    # firmware linear chain helpers (reported T -> Vin)
    G, Voff = frontend(args)
    args.k1 = lambda: 1023.0 / args.vref * args.fw_scale * G
    args.k0 = lambda: -1023.0 / args.vref * args.fw_scale * Voff + args.fw_offset

    # sensor curve
    src = open(args.rt_table).read() if args.rt_table else CURVE_F_10K
    rt = pd.read_csv(io.StringIO(src))
    rt = rt.sort_values("temp_C")
    ft, lnR = rt.temp_C.values, np.log(rt.resistance_ohm.values)
    T_of_R = lambda R: np.interp(np.log(R), lnR[::-1], ft[::-1])
    R_of_T = lambda T: np.exp(np.interp(T, ft, lnR))

    mon = load_monitor(args.monitor)
    ref = load_reference(args.ref, args.ref_channel)
    lag, corr = align(mon, ref)
    print(f"alignment: monitor + {lag:+.0f} s = reference   (corr {corr:.4f})")
    if corr < 0.95:
        print("WARN: correlation < 0.95 - check the two logs are the same run")

    mon = recover_raw(mon, args)
    ss = steady_points(mon, ref, lag, args.slope_limit, args.min_since_sp)
    print(f"quasi-steady points: {len(ss)} of {len(mon)}, "
          f"reference span {ss.Tref.min():.1f} to {ss.Tref.max():.1f} C")
    if len(ss) < 30:
        sys.exit("too few steady points - lengthen the holds and rerun")

    # fit: deg-N correction on the datasheet-curve temperature, bin means
    ss["Tds"] = T_of_R(ss.R.values)
    ss["bin"] = (ss.Tref / args.bin).round()
    b = ss.groupby("bin").agg(Tref=("Tref", "mean"), Tds=("Tds", "mean"),
                              n=("Tref", "size")).reset_index()
    gaps = [(a, c) for a, c in zip(np.sort(b.Tref)[:-1], np.sort(b.Tref)[1:]) if c - a > 8]
    coef = np.polyfit(b.Tds, b.Tref - b.Tds, args.deg, w=np.sqrt(b.n))
    lo_v, hi_v = ss.Tref.min(), ss.Tref.max()
    corr_of = lambda T: np.polyval(coef, np.clip(T, lo_v, hi_v))

    res = (ss.Tds + corr_of(ss.Tds)) - ss.Tref
    res0 = ss.Tds - ss.Tref
    print(f"residual, datasheet curve alone : max {res0.abs().max():.2f}  rms {np.sqrt((res0**2).mean()):.2f} C")
    print(f"residual, curve + correction    : max {res.abs().max():.2f}  rms {np.sqrt((res**2).mean()):.2f} C")
    print("correction coefficients (highest power first):",
          ", ".join(f"{c:.8e}" for c in coef))
    if gaps:
        print("UNSUPPORTED GAPS >8 C (datasheet shape only, no data): "
              + ", ".join(f"{a:.1f}-{c:.1f}" for a, c in gaps))

    # table: datasheet curve + clamped correction at --step spacing
    rows = []
    for T in np.arange(args.table_range[0], args.table_range[1] + 1e-9, args.step):
        raw = raw_from_R(R_of_T(T), args)
        if 1 <= raw <= 1022:
            rows.append((raw, T + corr_of(T)))
    rows.sort()
    n = len(rows)
    c_raw = ", ".join(f"{round(r):4d}" for r, _ in rows)
    c_t10 = ", ".join(f"{round(t*10):5d}" for _, t in rows)
    table = (f"#define NTC_LUT_N {n}\n"
             f"static const uint16_t NTC_LUT_RAW[NTC_LUT_N] PROGMEM = {{\n"
             f"  {c_raw} }};\n"
             f"static const int16_t  NTC_LUT_T10[NTC_LUT_N] PROGMEM = {{  /* 0.1 C units */\n"
             f"  {c_t10} }};\n")
    print("\n" + table)
    with open(args.out + "_table.h", "w") as f:
        f.write(f"/* generated by fit_ntc_lut.py - monitor {os.path.basename(args.monitor)},"
                f" ref {os.path.basename(args.ref)},"
                f" validated {lo_v:.1f} to {hi_v:.1f} C, clamped outside */\n" + table)
    print(f"table written to {args.out}_table.h - paste into NTC.h, "
          f"update its provenance block, then defaults + save on the unit")

    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(ss.Tref, ss.Tcold_C, "r.", ms=4, label="firmware during run")
        ax[0].plot(ss.Tref, ss.Tds + corr_of(ss.Tds), "b.", ms=4, label="curve + correction")
        lim = [ss.Tref.min() - 5, ss.Tref.max() + 5]
        ax[0].plot(lim, lim, "k--", lw=1, label="ideal")
        ax[0].set_ylabel("reported (C)"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
        ax[0].set_title(f"NTC vs reference, quasi-steady points (n={len(ss)})")
        ax[1].axhline(0, color="k", lw=1); ax[1].axhspan(-2, 2, color="green", alpha=.08)
        ax[1].plot(ss.Tref, res0, "r.", ms=4); ax[1].plot(ss.Tref, res, "b.", ms=4)
        for a, c in gaps: ax[1].axvspan(a, c, color="grey", alpha=.12)
        ax[1].set_xlabel("reference (C)"); ax[1].set_ylabel("error (C)"); ax[1].grid(alpha=.3)
        plt.tight_layout(); plt.savefig(args.out + ".png", dpi=130)
        print(f"plot written to {args.out}.png")
    except ImportError:
        print("matplotlib not installed - plot skipped")

if __name__ == "__main__":
    main()
