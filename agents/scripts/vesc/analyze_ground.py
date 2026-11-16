# Summarize a ground_test log: per launch, time to target, overshoot, settling and peak current.
import sys, re
rows = []
for line in open(sys.argv[1]):
    m = re.match(r"TEST (\S+) phase=(\S+) rpm=(\S+) i_motor=(\S+) i_in=(\S+) duty=(\S+) v_in=(\S+) t_mos=(\S+) fault=(\S+)", line)
    if m:
        t, ph, rpm, im, ii, du, v, tm, f = m.groups()
        rows.append((float(t), ph, float(rpm), float(im), float(ii), float(du), float(v), float(tm), f))
print("samples", len(rows), "faults", {r[8] for r in rows}, "v_in min %.2f" % min(r[6] for r in rows), "t_mos max %.1f" % max(r[7] for r in rows))
for ph, target in (("rpm2500", 2500), ("rpm4500", 4500)):
    p = [r for r in rows if r[1] == ph]
    if not p: continue
    t0 = p[0][0]
    rise = next((r[0] - t0 for r in p if r[2] >= 0.9 * target), None)
    peak = max(r[2] for r in p)
    neg = sum(1 for r in p if r[2] < -50)
    late = [r for r in p if r[0] - t0 > 0.8]
    import statistics as st
    rl = [r[2] for r in late]
    # jitter: sign changes of rpm slope during the launch's first 0.8 s
    early = [r[2] for r in p if r[0] - t0 <= 0.8]
    d = [b - a for a, b in zip(early, early[1:]) if abs(b - a) > 30]
    flips = sum(1 for a, b in zip(d, d[1:]) if a * b < 0)
    print(f"{ph}: rise90 {rise if rise is None else round(rise,3)} s, peak {peak:.0f} ({100*(peak-target)/target:+.0f}%), "
          f"backwards samples {neg}, slope flips first 0.8s {flips}, "
          f"steady mean {st.mean(rl):.0f} sd {st.pstdev(rl):.0f}, "
          f"i_motor max {max(r[3] for r in p):.1f} min {min(r[3] for r in p):.1f} sd {st.pstdev([r[3] for r in p]):.1f}")
    print("   first 0.6 s rpm:", " ".join(f"{r[2]:.0f}" for r in p if r[0]-t0 <= 0.6))
    print("   first 0.6 s i_m:", " ".join(f"{r[3]:.0f}" for r in p if r[0]-t0 <= 0.6))
