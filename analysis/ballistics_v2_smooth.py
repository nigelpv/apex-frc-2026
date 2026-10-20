"""Smooth shot map for APEX v2.

ballistics_v2.py picks the best hood angle at each distance on its own, and Monte-Carlo noise makes
the pick jump around (70 deg at 2.5 m, 60 deg at 2.75 m). Interpolating between rows like that
gives in-between shots nobody tested. This version:

  1. scores every (distance, angle) cell with common random numbers (the same 150 perturbed shots
     are used for every angle at a distance, so angles are compared fairly), and saves the grid;
  2. picks the angle curve with dynamic programming: maximise sum(P_make - 0.04 * tof) minus a
     penalty for changing angle between rows, with the angle never increasing with distance.

The result is a table the robot can interpolate. Writes shot_grid_v2.json, shot_map_v2.json and
prints Java rows for ShotTables.java.
"""
import json
import math
import sys
from multiprocessing import Pool

import numpy as np
import ballistics as b
import ballistics_v2 as v2

b.RELEASE_Z = 0.462
DISTS = [round(float(x), 2) for x in np.arange(1.25, 7.01, 0.25)]
ANGLES = [round(float(x), 1) for x in np.arange(45.0, 70.01, 2.5)]
N = 150
STEP_PENALTY = 0.012  # per 2.5 deg change between neighbouring rows
TOF_WEIGHT = 0.04


def draws(d):
    rng = np.random.default_rng(int(d * 1000) + 11)
    return [
        dict(dv=rng.normal(0, 0.02), dth=rng.normal(0, 0.5), vr=rng.normal(0, 0.15),
             m=rng.uniform(0.203, 0.227), cd=1 + rng.normal(0, 0.06), sp=0.10 + rng.normal(0, 0.04))
        for _ in range(N)
    ]


def cell(args):
    d, th = args
    v, tof, apex, traj = b.solve_speed(d, th, z0=b.RELEASE_Z)
    if tof is None or not b.rim_clear(d, traj) or apex > 4.6:
        return dict(d=d, th=th, ok=False)
    hits = 0
    for s in draws(d):
        ts, xs, zs = b.simulate(v * (1 + s["dv"]), th + s["dth"], spin_ratio=s["sp"], mass=s["m"],
                                cd=b.CD * s["cd"], z0=b.RELEASE_Z, x_stop=d + 3, vx_robot=s["vr"],
                                dt=0.004)
        xc, _ = b.crossing(ts, xs, zs)
        if xc is not None and abs(xc - d) < b.SAFE_HALF and b.rim_clear(d, (ts, xs, zs)):
            hits += 1
    return dict(d=d, th=th, ok=True, v=round(v, 3), tof=round(tof, 3), apex=round(apex, 2),
                p=round(hits / N, 3))


def grid():
    jobs = [(d, th) for d in DISTS for th in ANGLES]
    out = []
    with Pool(2) as pool:
        for i, r in enumerate(pool.imap(cell, jobs, chunksize=2)):
            out.append(r)
            if r["ok"]:
                print(f"d={r['d']:4.2f} th={r['th']:4.1f} p={r['p']:.3f} tof={r['tof']:.3f}", flush=True)
    return out


def smooth(cells):
    """DP over rows: state = angle index, angle non-increasing with distance."""
    table = {(c["d"], c["th"]): c for c in cells if c["ok"]}
    dists = [d for d in DISTS if any((d, th) in table for th in ANGLES)]
    nd, na = len(dists), len(ANGLES)
    NEG = -1e9
    score = np.full((nd, na), NEG)
    back = np.zeros((nd, na), dtype=int)
    for j, th in enumerate(ANGLES):
        c = table.get((dists[0], th))
        if c:
            score[0, j] = c["p"] - TOF_WEIGHT * c["tof"]
    for i in range(1, nd):
        for j, th in enumerate(ANGLES):
            c = table.get((dists[i], th))
            if not c:
                continue
            gain = c["p"] - TOF_WEIGHT * c["tof"]
            best, arg = NEG, 0
            for k in range(j, na):  # previous row angle >= this row angle
                if score[i - 1, k] <= NEG / 2:
                    continue
                s = score[i - 1, k] - STEP_PENALTY * (k - j)
                if s > best:
                    best, arg = s, k
            if best > NEG / 2:
                score[i, j] = best + gain
                back[i, j] = arg
    j = int(np.argmax(score[-1]))
    path = [j]
    for i in range(nd - 1, 0, -1):
        j = int(back[i, j])
        path.append(j)
    path.reverse()
    rows = []
    for i, j in enumerate(path):
        c = table[(dists[i], ANGLES[j])]
        rpm = c["v"] / v2.EXIT_EFF / (math.pi * v2.WHEEL_D) * 60
        rows.append(dict(dist=dists[i], launch_deg=ANGLES[j], exit_mps=c["v"], tof_s=c["tof"],
                         apex_m=c["apex"], p_make=c["p"], rpm=round(rpm)))
    return rows


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--reuse":
        cells = json.load(open("shot_grid_v2.json"))
    else:
        cells = grid()
        json.dump(cells, open("shot_grid_v2.json", "w"), indent=0)
    rows = smooth(cells)
    json.dump(rows, open("shot_map_v2.json", "w"), indent=1)
    best_free = {}
    for c in cells:
        if c["ok"] and c["p"] > best_free.get(c["d"], -1):
            best_free[c["d"]] = c["p"]
    print("\nSmoothed table (P = make probability, Pmax = best single angle at that distance)")
    for r in rows:
        print(f"d={r['dist']:4.2f} launch={r['launch_deg']:4.1f} v={r['exit_mps']:5.2f} tof={r['tof_s']:4.2f} "
              f"apex={r['apex_m']:4.2f} P={r['p_make']:.2f} Pmax={best_free[r['dist']]:.2f} rpm={r['rpm']}")
    print("\n// HUB rows {dist, launchDeg, exitMps, tof}")
    for r in rows:
        print(f"    {{{r['dist']:.2f}, {r['launch_deg']:.1f}, {r['exit_mps']:.3f}, {r['tof_s']:.3f}}},")
