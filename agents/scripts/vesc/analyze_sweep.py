# Summarize a current_sweep log: per launch, how fast and how cleanly the motor got going.
import sys, re
rows = []
for line in open(sys.argv[1]):
    m = re.match(r"TEST (\S+) phase=(\S+) rpm=(\S+) i_motor=(\S+) i_in=\S+ duty=(\S+) v_in=(\S+) t_mos=(\S+) fault=(\S+)", line)
    if m:
        t, ph, rpm, im, du, v, tm, f = m.groups()
        rows.append((float(t), ph, float(rpm), float(im), float(du), float(v), float(tm), f))
print("samples", len(rows), "faults", {r[7] for r in rows}, "v_in min %.2f" % min(r[5] for r in rows),
      "t_mos max %.1f" % max(r[6] for r in rows))
for ph in sorted({r[1] for r in rows if r[1].startswith("cur")}, key=lambda s: int(s[3:])):
    p = [r for r in rows if r[1] == ph]
    t0 = p[0][0]
    rpm = [r[2] for r in p]
    def first(th):
        return next((round(r[0] - t0, 2) for r in p if r[2] >= th), None)
    # stalls: drops of more than 200 ERPM between samples while driving
    drops = sum(1 for a, b in zip(rpm, rpm[1:]) if a - b > 200)
    print(f"{ph}: peak {max(rpm):.0f}, t>1000 {first(1000)} s, t>2000 {first(2000)} s, t>4000 {first(4000)} s, "
          f"drops>200 {drops}, i_motor mean {sum(r[3] for r in p)/len(p):.1f} A, end rpm {rpm[-1]:.0f}")
    print("   rpm:", " ".join(f"{x:.0f}" for x in rpm))
