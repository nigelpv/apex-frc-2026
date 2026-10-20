"""Agent-based REBUILT match simulator used to choose APEX's architecture.

Every FUEL is tracked individually on a 2-D field. Six robots follow
archetype-specific policies through AUTO, TRANSITION, the four ALLIANCE SHIFTS
and END GAME. Scored FUEL is recycled out of the HUB exits into the NEUTRAL
ZONE (the core economy of REBUILT), so alliances compete for a finite supply.

The model is deliberately simple about driving (straight lines, zone gates at
the TRENCH lanes / BUMPS, no robot-robot collisions) and detailed about the
things that decide REBUILT: fuel supply, intake rate, hopper capacity, firing
rate, shooting while moving, shift timing and recycling.
"""
from __future__ import annotations
import math, dataclasses, random
import numpy as np
from field import (FIELD_LENGTH as FL, FIELD_WIDTH as FW, BLUE_HUB, RED_HUB,
                   ALLIANCE_ZONE_DEPTH as AZ, NZ_BOX, CENTER_X, TRENCH_LANES_Y)

DT = 0.05
AUTO_T, TELEOP_T = 20.0, 140.0
HUB_DELAY = 1.2           # s from entering the hub to leaving an exit
GRACE = 3.0               # scoring assessment continues 3 s after deactivation


@dataclasses.dataclass
class Archetype:
    name: str
    cap: int                 # max FUEL stored
    trench_cap: int          # max FUEL while still fitting under the TRENCH
    shoot_bps: float         # sustained firing rate
    intake_bps: float        # max sustained intake rate
    intake_w: float          # effective intake width (m)
    vmax: float              # top speed (m/s)
    v_collect: float         # speed while sweeping fuel
    sotm: str                # "none", "heading" (fixed shooter, heading-locked), "turret"
    align_s: float           # time to settle before a stationary volley
    accuracy: float          # make probability at typical ranges
    passer: bool = False     # lobs fuel into own zone while collecting in inactive shift
    bump_s: float = 1.2      # extra time to cross a BUMP


ARCH = {
    # Einstein-level reference robots (from published specs, see docs/research.md)
    "dumper254": Archetype("4-wide dumper (254-class)", 78, 60, 27, 18, 0.66, 4.3, 2.6, "none", 0.7, 0.93),
    "dumper2910": Archetype("hi-BPS dumper (2910-class)", 62, 62, 33, 20, 0.66, 4.4, 2.6, "none", 0.7, 0.92),
    "turret4414": Archetype("turret+rotor (4414-class)", 60, 60, 20, 16, 0.62, 4.3, 2.6, "turret", 0.0, 0.93),
    "big2481": Archetype("115-ball bump dumper (2481-class)", 115, 0, 20, 18, 0.72, 4.0, 2.4, "none", 0.8, 0.92),
    "partner": Archetype("solid partner", 50, 50, 14, 10, 0.55, 4.0, 2.2, "none", 0.9, 0.85),
    # Our design
    "apex": Archetype("APEX", 108, 72, 36, 26, 0.70, 4.4, 2.8, "heading", 0.25, 0.93, passer=True),
    "apex_nopass": Archetype("APEX (no passing)", 108, 72, 36, 26, 0.70, 4.4, 2.8, "heading", 0.25, 0.93),
    "apex_small": Archetype("APEX w/ 72 cap only", 72, 72, 36, 26, 0.70, 4.4, 2.8, "heading", 0.25, 0.93, passer=True),
    "apex_noSOTM": Archetype("APEX w/o SOTM", 108, 72, 36, 26, 0.70, 4.4, 2.8, "none", 0.7, 0.93, passer=True),
}


def hub_active(alliance, t, red_won_auto):
    """t is match time since start (0..160)."""
    if t < AUTO_T:
        return True
    tt = t - AUTO_T
    if tt < 10 or tt >= 110:
        return True
    shift = int((tt - 10) // 25)          # 0..3
    first_inactive = "red" if red_won_auto else "blue"
    inactive_on_even = (alliance == first_inactive)
    return (shift % 2 == 1) if inactive_on_even else (shift % 2 == 0)


class Robot:
    def __init__(self, arch: Archetype, alliance: str, start_y: float, rng):
        self.a = arch
        self.alliance = alliance
        self.sign = 1 if alliance == "blue" else -1
        x0 = AZ - 0.55 if alliance == "blue" else FL - AZ + 0.55
        self.p = np.array([x0, start_y])
        self.load = 8
        self.intake_budget = 0.0
        self.shoot_budget = 0.0
        self.align_left = 0.0
        self.scored = 0
        self.shot = 0
        self.target = None
        self.mode = "collect"
        self.rng = rng
        self.cross_delay = 0.0
        self.aligned = False
        self.moving_shot = False

    # --- geometry helpers ---
    def own_x(self, x):           # mirror into "blue" frame
        return x if self.alliance == "blue" else FL - x

    def in_alliance_zone(self, p=None):
        p = self.p if p is None else p
        return self.own_x(p[0]) <= AZ + 0.3     # bumpers partially in the zone

    def hub(self):
        return np.array(BLUE_HUB if self.alliance == "blue" else RED_HUB)


class Match:
    def __init__(self, blue, red, n_fuel=504, seed=0, defense=False, steal=True):
        self.steal = steal
        self.rng = np.random.default_rng(seed)
        r = self.rng
        ys = [1.2, 4.0, 6.8]
        self.robots = [Robot(ARCH[k], "blue", ys[i], r) for i, k in enumerate(blue)] + \
                      [Robot(ARCH[k], "red", ys[i], r) for i, k in enumerate(red)]
        pre = 48
        nz = n_fuel - pre - 4 * 24
        x0, x1, y0, y1 = NZ_BOX
        xs = r.uniform(x0, x1, nz); ys_ = r.uniform(y0, y1, nz)
        fuel = [np.stack([xs, ys_], 1)]
        for depot_x, depot_y in ((0.45, 5.9), (FL - 0.45, FW - 5.9)):
            fuel.append(np.stack([r.uniform(depot_x - .3, depot_x + .3, 24),
                                  r.uniform(depot_y - .5, depot_y + .5, 24)], 1))
        self.fuel = np.concatenate(fuel)                 # on-carpet fuel positions
        self.chute = {"blue": 24, "red": 24}              # human player releases over time
        self.in_flight = []   # (t_arrive, alliance, robot, made, kind, landing)
        self.score = {"blue": 0, "red": 0}
        self.auto_fuel = {"blue": 0, "red": 0}
        self.red_won_auto = None
        self.defense = defense

    # ------------------------------------------------------------------
    def active(self, alliance, t):
        if t < AUTO_T + 0.0 or self.red_won_auto is None:
            return True
        return hub_active(alliance, t, self.red_won_auto)

    def counts(self, alliance, t_enter):
        """Fuel entering at t_enter counts if the hub was active within GRACE s."""
        if t_enter < AUTO_T + GRACE:
            return True
        return any(self.active(alliance, t_enter - g) for g in (0, GRACE * 0.99))

    def time_to_next_active(self, alliance, t):
        tt = t
        while tt < 160 and not self.active(alliance, tt):
            tt += 0.25
        return tt - t

    def time_left_active(self, alliance, t):
        tt = t
        while tt < 160 and self.active(alliance, tt):
            tt += 0.25
        return tt - t

    # ------------------------------------------------------------------
    def densest(self, robot, region, t):
        """Pick a collection target: highest fuel count in a 1.2 m disc, discounted by distance."""
        if len(self.fuel) == 0:
            return None
        f = self.fuel
        if region == "own":
            m = np.array([robot.in_alliance_zone(q) for q in f])
        else:  # neutral zone (both halves; centre line only matters in AUTO)
            ox = np.where(robot.sign > 0, f[:, 0], FL - f[:, 0])
            m = (ox > AZ + 0.7) & (ox < FL - AZ - 0.7)
            if self.steal:   # inactive robots also raid the opponent ALLIANCE ZONE stash
                m |= ox > FL - AZ
            if t < AUTO_T:
                m &= ox < CENTER_X - 0.3
        cand = f[m]
        if len(cand) == 0:
            return None
        idx = self.rng.choice(len(cand), size=min(60, len(cand)), replace=False)
        best, bv = None, -1
        for c in cand[idx]:
            n = np.sum(np.hypot(cand[:, 0] - c[0], cand[:, 1] - c[1]) < 1.2)
            d = np.hypot(*(c - robot.p))
            v = n / (1.0 + 0.35 * d)
            if v > bv:
                bv, best = v, c
        return best

    def path_time(self, robot, a, b):
        """Straight-line time plus a zone-crossing penalty when going alliance<->neutral."""
        d = np.hypot(*(b - a))
        cross = robot.in_alliance_zone(a) != robot.in_alliance_zone(b)
        pen = 0.0
        if cross:
            if robot.load <= robot.a.trench_cap:
                ylane = min(TRENCH_LANES_Y, key=lambda y: abs(y - a[1]) + abs(y - b[1]))
                d = abs(a[1] - ylane) + abs(b[1] - ylane) + abs(a[0] - b[0])
            else:
                pen = robot.a.bump_s
        return d / robot.a.vmax + pen

    # ------------------------------------------------------------------
    def step_robot(self, R: Robot, t):
        A = R.a
        act = self.active(R.alliance, t)
        dt = DT
        if R.cross_delay > 0:
            R.cross_delay -= dt
            return
        home = R.hub() + np.array([-R.sign * 2.2, 0.0])     # default shooting spot (2.8 m)
        # --- decide mode -------------------------------------------------
        if act:
            t_left = self.time_left_active(R.alliance, t) + GRACE * 0.6
            own_near = None
            if A.sotm == "none" and R.load < 20:
                own_near = self.densest(R, "own", t)
                if own_near is not None and np.hypot(*(own_near - R.p)) > 2.5:
                    own_near = None
            if own_near is not None and R.load < A.cap:
                R.mode, R.target = "collect_own", own_near
            elif R.load > 0:
                R.mode = "score"
            else:
                # empty: collect where fuel is reachable and can be scored before window closes
                own_t = self.densest(R, "own", t)
                if own_t is not None:
                    R.mode, R.target = "collect_own", own_t
                else:
                    nt = self.densest(R, "neutral", t)
                    if nt is not None and t_left > self.path_time(R, R.p, nt) * 2 + 2.5:
                        R.mode, R.target = "collect_nz", nt
                    else:
                        R.mode, R.target = "wait", home
        else:
            t_next = self.time_to_next_active(R.alliance, t)
            back = self.path_time(R, R.p, home)
            if t_next <= back + 0.6 or R.load >= A.cap:
                R.mode, R.target = "return", home
            else:
                nt = self.densest(R, "neutral", t)
                R.mode, R.target = ("collect_nz", nt) if nt is not None else ("return", home)

        # --- act ---------------------------------------------------------
        moving = False
        R.moving_shot = False
        if R.mode == "score":
            if not R.in_alliance_zone():
                self.move(R, home, A.vmax)
                moving = True
                R.aligned = False
            else:
                can_move_while = A.sotm in ("turret", "heading")
                if can_move_while:
                    # sweep own-zone fuel while firing
                    tgt = self.densest(R, "own", t)
                    if tgt is not None and R.load < A.cap:
                        ok = True
                        if A.sotm == "heading":   # intake only when driving away from the hub
                            away = R.p - R.hub(); mv = tgt - R.p
                            ok = np.dot(away, mv) > 0.3 * np.linalg.norm(away) * np.linalg.norm(mv)
                        if ok:
                            self.move(R, tgt, A.v_collect)
                            moving = True
                            R.moving_shot = True
                            self.intake(R)
                    R.align_left = 0
                if not can_move_while and not R.aligned:
                    if R.align_left <= 0:
                        R.align_left = A.align_s
                    R.align_left -= dt
                    if R.align_left <= 0:
                        R.aligned = True
                else:
                    self.fire(R, t, at_hub=True)
        elif R.mode in ("collect_own", "collect_nz"):
            if R.target is not None:
                self.move(R, R.target, A.v_collect if np.hypot(*(R.target - R.p)) < 1.0 else A.vmax)
                moving = True
                self.intake(R)
            # passers lob surplus into their own zone while in the neutral zone
            if A.passer and not act and not R.in_alliance_zone() and R.load > 12:
                self.fire(R, t, at_hub=False)
        elif R.mode in ("return", "wait"):
            self.move(R, R.target, A.vmax)
            moving = True
            if R.load < A.cap:
                self.intake(R)
        if not moving:
            R.intake_budget = 0
        elif R.a.sotm == "none" and R.mode != "score":
            R.aligned = False

    def move(self, R, tgt, v):
        d = tgt - R.p
        n = np.hypot(*d)
        if n < 1e-6:
            return
        step = min(n, v * DT)
        newp = R.p + d / n * step
        # zone crossing costs time (trench detour or bump)
        if R.in_alliance_zone(newp) != R.in_alliance_zone(R.p):
            if R.load > R.a.trench_cap:
                R.cross_delay = R.a.bump_s
            else:
                R.cross_delay = 0.35
        R.p = np.clip(newp, [0.4, 0.4], [FL - 0.4, FW - 0.4])

    def intake(self, R):
        A = R.a
        R.intake_budget = min(R.intake_budget + A.intake_bps * DT, 3)
        if R.load >= A.cap or len(self.fuel) == 0:
            return
        d = np.hypot(self.fuel[:, 0] - R.p[0], self.fuel[:, 1] - R.p[1])
        near = np.where(d < A.intake_w / 2 + 0.45)[0]
        if len(near) == 0:
            return
        take = int(min(len(near), R.intake_budget, A.cap - R.load))
        if take <= 0:
            return
        sel = near[np.argsort(d[near])[:take]]
        self.fuel = np.delete(self.fuel, sel, axis=0)
        R.load += take
        R.intake_budget -= take

    def fire(self, R, t, at_hub):
        A = R.a
        R.shoot_budget = min(R.shoot_budget + A.shoot_bps * DT, 4)
        n = int(min(R.load, R.shoot_budget))
        if n <= 0:
            return
        R.load -= n
        R.shoot_budget -= n
        if at_hub:
            acc = A.accuracy - (0.05 if R.moving_shot else 0.0)
            for _ in range(n):
                made = self.rng.random() < acc
                self.in_flight.append((t + 1.0, R.alliance, R, made, "hub"))
                R.shot += 1
        else:  # pass toward own zone corner region
            for _ in range(n):
                land = np.array([R.own_x(float(np.clip(1.6 + self.rng.normal(0, 0.8), 0.3, 3.8))),
                                 np.clip(self.rng.normal(4.0, 1.6), 0.5, FW - 0.5)])
                self.in_flight.append((t + 1.2, R.alliance, R, False, ("land", land)))

    def resolve_flight(self, t):
        keep = []
        new_fuel = []
        for item in self.in_flight:
            ta, al, R, made, kind = item
            if ta > t:
                keep.append(item); continue
            if kind == "hub" and made:
                if self.counts(al, ta):
                    self.score[al] += 1
                    R.scored += 1
                    if ta < AUTO_T + GRACE:
                        self.auto_fuel[al] += 1
                # recycle out of the hub exits into the neutral zone after a delay
                keep.append((ta + HUB_DELAY, al, R, False, "exit"))
            elif kind == "exit":
                hub = np.array(BLUE_HUB if al == "blue" else RED_HUB)
                sgn = 1 if al == "blue" else -1
                new_fuel.append([hub[0] + sgn * self.rng.uniform(0.9, 2.6),
                                 hub[1] + self.rng.uniform(-1.6, 1.6)])
            elif kind == "hub":   # miss: bounce near the hub
                hub = np.array(BLUE_HUB if al == "blue" else RED_HUB)
                ang = self.rng.uniform(0, 2 * math.pi)
                new_fuel.append(hub + 1.0 * np.array([math.cos(ang), math.sin(ang)]))
            else:   # passed fuel lands
                new_fuel.append(kind[1])
        self.in_flight = keep
        if new_fuel:
            self.fuel = np.concatenate([self.fuel, np.array(new_fuel)])

    def human_player(self, t):
        # chute fuel trickles into each alliance zone near the OUTPOST (2/s)
        for al, x in (("blue", 0.6), ("red", FL - 0.6)):
            if self.chute[al] > 0 and self.rng.random() < 2 * DT:
                self.chute[al] -= 1
                self.fuel = np.concatenate([self.fuel, [[x + self.rng.normal(0, .3),
                                                         0.9 + self.rng.normal(0, .3)]]])

    def run(self):
        t = 0.0
        while t < 160.0 + GRACE:
            if self.red_won_auto is None and t >= AUTO_T:
                b, r = self.auto_fuel["blue"], self.auto_fuel["red"]
                self.red_won_auto = (r > b) if r != b else bool(self.rng.random() < 0.5)
            if t < 160.0:
                self.human_player(t)
                for R in self.robots:
                    self.step_robot(R, t)
            self.resolve_flight(t)
            t += DT
        return self


def play(blue, red, n=6, n_fuel=504, steal=True):
    res = []
    for s in range(n):
        m = Match(blue, red, n_fuel=n_fuel, seed=s, steal=steal).run()
        res.append((m.score["blue"], m.score["red"], [R.scored for R in m.robots],
                    m.auto_fuel["blue"], m.auto_fuel["red"]))
    b = np.mean([r[0] for r in res]); rr = np.mean([r[1] for r in res])
    per = np.mean([r[2] for r in res], axis=0)
    ab = np.mean([r[3] for r in res]); ar = np.mean([r[4] for r in res])
    return b, rr, per, ab, ar


if __name__ == "__main__":
    import sys
    scen = [
        ("Champs clone alliances", ["turret4414", "dumper254", "dumper2910"], ["turret4414", "dumper254", "dumper2910"]),
        ("APEX replaces the 4414-class robot", ["apex", "dumper254", "dumper2910"], ["turret4414", "dumper254", "dumper2910"]),
        ("APEX replaces the 254-class robot", ["turret4414", "apex", "dumper2910"], ["turret4414", "dumper254", "dumper2910"]),
        ("APEX + 2 partners vs 4414 + 2 partners", ["apex", "partner", "partner"], ["turret4414", "partner", "partner"]),
        ("APEX + 2 partners vs 254 + 2 partners", ["apex", "partner", "partner"], ["dumper254", "partner", "partner"]),
        ("APEX + 2 partners vs 2481 + 2 partners", ["apex", "partner", "partner"], ["big2481", "partner", "partner"]),
        ("Ablation: no passing", ["apex_nopass", "partner", "partner"], ["turret4414", "partner", "partner"]),
        ("Ablation: 72 cap only", ["apex_small", "partner", "partner"], ["turret4414", "partner", "partner"]),
        ("Ablation: no SOTM", ["apex_noSOTM", "partner", "partner"], ["turret4414", "partner", "partner"]),
        ("3x APEX vs Champs clones", ["apex", "apex", "apex"], ["turret4414", "dumper254", "dumper2910"]),
    ]
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    nf = int(sys.argv[2]) if len(sys.argv) > 2 else 504
    for name, b, r in scen:
        bs, rs, per, ab, ar = play(b, r, n=n, n_fuel=nf)
        print(f"{name:42s} blue {bs:6.1f} (auto {ab:5.1f}) | red {rs:6.1f} (auto {ar:5.1f}) | "
              f"per-robot {np.round(per,1)}")
