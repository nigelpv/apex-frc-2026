"""Shot map for APEX v2 (CAD release point and hood range).

Same physics as ballistics.py (quadratic drag + Magnus, RK4, Monte-Carlo make probability), but
with the v2 geometry: ball released at 18.2 in (flywheel contact patch, turret at 13.875 in deck
height), hood launch angle 45-70 deg, single 4 in flywheel with a fixed hood (exit speed ~0.47 x
wheel surface speed). Writes shot_map_v2.json and pass_map_v2.json and prints Java rows for
ShotTables.java.
"""
import json
import math
import numpy as np
import ballistics as b

b.RELEASE_Z = 0.462
WHEEL_D = 0.1016
EXIT_EFF = 0.47


def run():
    rows = []
    rng = np.random.default_rng(7)
    for d in np.arange(1.25, 7.01, 0.25):
        best = None
        for th in np.arange(45.0, 70.01, 2.5):
            v, tof, apex, traj = b.solve_speed(d, th, z0=b.RELEASE_Z)
            if tof is None or not b.rim_clear(d, traj) or apex > 4.6:
                continue
            p = make_p(d, th, v, rng)
            key = p - 0.04 * tof
            if best is None or key > best[0]:
                best = (key, th, v, tof, apex, p)
        if best:
            _, th, v, tof, apex, p = best
            rpm = v / EXIT_EFF / (math.pi * WHEEL_D) * 60
            rows.append(dict(dist=round(float(d), 2), launch_deg=float(th), exit_mps=round(v, 3),
                             tof_s=round(tof, 3), apex_m=round(apex, 2), p_make=round(p, 3), rpm=round(rpm)))
            print(f"d={d:4.2f}  launch={th:4.1f}  v={v:5.2f}  tof={tof:4.2f}  apex={apex:4.2f}  P={p:4.2f}  rpm={rpm:5.0f}", flush=True)
    return rows


def make_p(dist, theta, v, rng, n=120):
    hits = 0
    for _ in range(n):
        dv = v * (1 + rng.normal(0, 0.02))
        dth = theta + rng.normal(0, 0.5)
        vr = rng.normal(0, 0.15)
        m = rng.uniform(0.203, 0.227)
        cd = b.CD * (1 + rng.normal(0, 0.06))
        sp = 0.10 + rng.normal(0, 0.04)
        ts, xs, zs = b.simulate(dv, dth, spin_ratio=sp, mass=m, cd=cd, z0=b.RELEASE_Z, x_stop=dist + 3,
                                vx_robot=vr, dt=0.004)
        xc, _ = b.crossing(ts, xs, zs)
        if xc is not None and abs(xc - dist) < b.SAFE_HALF and b.rim_clear(dist, (ts, xs, zs)):
            hits += 1
    return hits / n


def pass_map():
    """Lob into our ALLIANCE ZONE: land at floor level (ball centre 0.3 m) at 4-11 m, 50 deg."""
    out = []
    for d in np.arange(4.0, 11.01, 1.0):
        v, tof, apex, _ = b.solve_speed(d, 50.0, z_plane=0.3, z0=b.RELEASE_Z)
        rpm = v / EXIT_EFF / (math.pi * WHEEL_D) * 60
        out.append(dict(dist=float(d), launch_deg=50.0, exit_mps=round(v, 3), tof_s=round(tof, 3),
                        apex_m=round(apex, 2), rpm=round(rpm)))
        print(f"pass d={d:4.1f}  v={v:5.2f}  tof={tof:4.2f}  apex={apex:4.2f}  rpm={rpm:5.0f}", flush=True)
    return out


if __name__ == "__main__":
    rows = run()
    json.dump(rows, open("shot_map_v2.json", "w"), indent=1)
    passes = pass_map()
    json.dump(passes, open("pass_map_v2.json", "w"), indent=1)
    print("\n// HUB rows {dist, launchDeg, exitMps, tof}")
    for r in rows:
        print(f"    {{{r['dist']:.2f}, {r['launch_deg']:.1f}, {r['exit_mps']:.3f}, {r['tof_s']:.3f}}},")
    print("// PASS rows")
    for r in passes:
        print(f"    {{{r['dist']:.2f}, {r['launch_deg']:.1f}, {r['exit_mps']:.3f}, {r['tof_s']:.3f}}},")
