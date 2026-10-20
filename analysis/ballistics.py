"""FUEL ballistics for the APEX drum shooter.

Model: point-mass sphere with quadratic drag and Magnus lift, RK4 integration.
For every shot distance we solve the exit speed needed to drop the ball through
the centre of the HUB opening for each launch angle, then Monte-Carlo the
dispersion (speed, angle, ball-mass, drag and spin variation) and pick the
launch angle that maximises make probability with a rim-clearance constraint.

Output: shot map (distance -> hood angle, exit speed, drum RPM, time of flight)
used directly by the robot code (ShotMap.java) and by the hood range spec.
"""
import math
import numpy as np
from field import HUB_OPENING_Z, HUB_OPENING_ACROSS, FUEL_D, FUEL_MASS, IN

G = 9.81
RHO = 1.20
CD = 0.47
R_BALL = FUEL_D / 2
AREA = math.pi * R_BALL ** 2

RELEASE_Z = 0.52            # m, ball centre at exit of drum/hood (21.5 in robot)
OPEN_HALF = HUB_OPENING_ACROSS / 2      # 0.53 m (flat-to-flat / 2)
SAFE_HALF = OPEN_HALF - R_BALL - 0.03   # usable half-window for ball centre


def cl_from_spin(s):
    """Lift coefficient vs spin ratio S = w r / v (Sawicki-style fit)."""
    s = abs(s)
    return s / (2.022 * s + 0.981) if s > 0 else 0.0


def simulate(v0, theta_deg, spin_ratio=0.10, mass=FUEL_MASS, cd=CD, z0=RELEASE_Z,
             dt=0.002, x_stop=10.0, vx_robot=0.0):
    """Integrate a shot. Returns arrays t, x, z. spin_ratio > 0 = backspin (lift)."""
    th = math.radians(theta_deg)
    vx, vz = v0 * math.cos(th) + vx_robot, v0 * math.sin(th)
    x, z, t = 0.0, z0, 0.0
    w_r = spin_ratio * v0                    # surface speed of spin, decays slowly
    k = 0.5 * RHO * AREA / mass
    xs, zs, ts = [x], [z], [t]

    def acc(vx, vz):
        v = math.hypot(vx, vz)
        cl = cl_from_spin(w_r / max(v, 1e-6))
        ax = -k * cd * v * vx - k * cl * v * vz      # lift perpendicular to v (backspin)
        az = -G - k * cd * v * vz + k * cl * v * vx
        return ax, az

    while z > 0 and x < x_stop and t < 4:
        a1 = acc(vx, vz)
        a2 = acc(vx + a1[0] * dt / 2, vz + a1[1] * dt / 2)
        a3 = acc(vx + a2[0] * dt / 2, vz + a2[1] * dt / 2)
        a4 = acc(vx + a3[0] * dt, vz + a3[1] * dt)
        x += dt * (vx + dt / 6 * (a1[0] + a2[0] + a3[0]))
        z += dt * (vz + dt / 6 * (a1[1] + a2[1] + a3[1]))
        vx += dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
        vz += dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
        t += dt
        w_r *= (1 - 0.05 * dt)
        xs.append(x); zs.append(z); ts.append(t)
    return np.array(ts), np.array(xs), np.array(zs)


def crossing(ts, xs, zs, z_plane=HUB_OPENING_Z):
    """Horizontal position / time where the ball descends through z_plane."""
    for i in range(1, len(zs)):
        if zs[i - 1] >= z_plane > zs[i] and zs[i] < zs[i - 1]:
            f = (zs[i - 1] - z_plane) / (zs[i - 1] - zs[i])
            return xs[i - 1] + f * (xs[i] - xs[i - 1]), ts[i - 1] + f * (ts[i] - ts[i - 1])
    return None, None


def apex_height(zs):
    return float(np.max(zs))


def solve_speed(dist, theta, z_plane=HUB_OPENING_Z, **kw):
    """Exit speed that drops the ball centre through z_plane at horizontal distance dist."""
    lo, hi = 2.0, 25.0
    for _ in range(45):
        mid = (lo + hi) / 2
        ts, xs, zs = simulate(mid, theta, x_stop=dist + 3, **kw)
        xc, _ = crossing(ts, xs, zs, z_plane)
        if xc is None:
            too_slow = zs.max() < z_plane      # never reached the plane
            if too_slow:
                lo = mid
            else:
                hi = mid                       # still above the plane past x_stop
        elif xc < dist:
            lo = mid
        else:
            hi = mid
    v = (lo + hi) / 2
    ts, xs, zs = simulate(v, theta, x_stop=dist + 3, **kw)
    xc, tc = crossing(ts, xs, zs, z_plane)
    return v, tc, apex_height(zs), (ts, xs, zs)


def rim_clear(dist, traj):
    """Ball centre must be above the front rim + radius where it passes the rim."""
    ts, xs, zs = traj
    x_rim = dist - OPEN_HALF
    i = np.searchsorted(xs, x_rim)
    if i >= len(xs):
        return False
    return zs[i] > HUB_OPENING_Z + R_BALL + 0.02


def make_probability(dist, theta, v, n=300, rng=None, spin=0.10):
    """Monte-Carlo dispersion: speed 2 %, angle 0.5 deg, mass, drag, spin, and a
    0.15 m/s robot-velocity estimate error (shoot-on-the-move) applied over the TOF."""
    rng = rng or np.random.default_rng(1)
    hits = 0
    for _ in range(n):
        dv = v * (1 + rng.normal(0, 0.02))
        dth = theta + rng.normal(0, 0.5)
        vr = rng.normal(0, 0.15)
        m = rng.uniform(0.203, 0.227)
        cd = CD * (1 + rng.normal(0, 0.06))
        sp = spin + rng.normal(0, 0.04)
        ts, xs, zs = simulate(dv, dth, spin_ratio=sp, mass=m, cd=cd, x_stop=dist + 3, vx_robot=vr)
        xc, _ = crossing(ts, xs, zs)
        if xc is not None and abs(xc - dist) < SAFE_HALF and rim_clear(dist, (ts, xs, zs)):
            hits += 1
    return hits / n


def shot_map(dists=None, angles=None, verbose=True):
    dists = dists if dists is not None else np.arange(1.25, 6.76, 0.25)
    angles = angles if angles is not None else np.arange(45, 81, 2.5)
    rows = []
    rng = np.random.default_rng(7)
    for d in dists:
        best = None
        for th in angles:
            v, tof, apex, traj = solve_speed(d, th)
            if tof is None or not rim_clear(d, traj) or apex > 4.6:
                continue
            p = make_probability(d, th, v, n=200, rng=rng)
            # make probability first; flatter/faster shots (short TOF) break near-ties,
            # because TOF multiplies every shoot-on-the-move velocity error
            key = p - 0.04 * tof
            if best is None or key > best[0]:
                best = (key, th, v, tof, apex, p)
        if best:
            _, th, v, tof, apex, p = best
            rows.append(dict(dist=round(float(d), 2), hood_deg=float(th), exit_mps=round(v, 3),
                             tof_s=round(tof, 3), apex_m=round(apex, 2), p_make=round(p, 3)))
            if verbose:
                print(f"d={d:4.2f} m  launch={th:5.1f} deg  v={v:5.2f} m/s  tof={tof:4.2f} s  "
                      f"apex={apex:4.2f} m  P(make)={p:4.2f}")
    return rows


# Drum shooter kinematics ---------------------------------------------------
DRUM_D = 4.0 * IN            # main drum diameter (0.1016 m)
EXIT_EFF = 0.88              # exit speed / drum surface speed with powered hood rollers


def drum_rpm_for(v_exit):
    surface = v_exit / EXIT_EFF
    return surface / (math.pi * DRUM_D) * 60


if __name__ == "__main__":
    rows = shot_map()
    vmax = max(r["exit_mps"] for r in rows)
    print(f"\nMax exit speed {vmax:.2f} m/s -> drum {drum_rpm_for(vmax):.0f} RPM (4 in drum)")
    print(f"Hood launch-angle span: {min(r['hood_deg'] for r in rows)}-"
          f"{max(r['hood_deg'] for r in rows)} deg")
    import json
    with open("shot_map.json", "w") as f:
        json.dump(rows, f, indent=1)
    # Passing lob: neutral zone (x~6.5) to our stash region (x~1.5): 5-7 m, land anywhere
    for d in (5.0, 7.0, 9.0):
        v, tof, apex, traj = solve_speed(d, 50, z_plane=0.3)
        print(f"pass {d} m @50deg: v={v:.2f} m/s apex={apex:.2f} m tof={tof:.2f}s  "
              f"drum {drum_rpm_for(v):.0f} RPM")
