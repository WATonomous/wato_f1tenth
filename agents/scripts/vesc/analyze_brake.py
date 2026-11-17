# Summarize a brake_test log: how fast the car slows from 4000 to 1500 ERPM and whether it brakes.
import sys, re
rows = []
for line in open(sys.argv[1]):
    m = re.match(r"TEST (\S+) phase=(\S+) rpm=(\S+) i_motor=(\S+) i_in=(\S+) duty=\S+ v_in=(\S+) t_mos=(\S+) fault=(\S+)", line)
    if m:
        t, ph, rpm, im, ii, v, tm, f = m.groups()
        rows.append((float(t), ph, float(rpm), float(im), float(ii), float(v), float(tm), f))
print("samples", len(rows), "faults", {r[7] for r in rows}, "v_in min %.2f max %.2f" % (min(r[5] for r in rows), max(r[5] for r in rows)))
up = [r for r in rows if r[1] == "rpm4000"]
dn = [r for r in rows if r[1] == "rpm1500"]
print("rpm4000: speed at end %.0f, mean last 0.5 s %.0f" % (up[-1][2], sum(r[2] for r in up if r[0] > 2.0) / max(1, sum(1 for r in up if r[0] > 2.0))))
t0 = dn[0][0]
reach = next((round(r[0] - t0, 2) for r in dn if r[2] <= 1700), None)
print(f"rpm1500: time to <=1700 {reach} s, min rpm {min(r[2] for r in dn):.0f}, min i_motor {min(r[3] for r in dn):.1f} A, "
      f"min i_in {min(r[4] for r in dn):.1f} A, mean last 0.5 s {sum(r[2] for r in dn if r[0]-t0 > 1.0)/max(1,sum(1 for r in dn if r[0]-t0 > 1.0)):.0f}")
print("   rpm:", " ".join(f"{r[2]:.0f}" for r in dn[:40]))
print("   i_m:", " ".join(f"{r[3]:.1f}" for r in dn[:40]))
