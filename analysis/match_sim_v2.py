"""Match simulator runs for APEX v2 as built in CAD (34 FUEL hopper, 20 FUEL/s turret feed).

Uses match_sim.py (agent-based REBUILT model; reference robots from published 2026 specs).
Caveat: the robot policy is simple. A robot whose shooting rate is below its intake rate keeps
sweeping its own ALLIANCE ZONE while firing and rarely goes back to the NEUTRAL ZONE, which is why
a 15 FUEL/s feed scores far less than 20 FUEL/s in this model.
Run: python3 match_sim_v2.py [matches per scenario]
"""
import sys
import match_sim as ms

A = ms.Archetype
ms.ARCH["apex_v2"] = A("APEX v2 (34 cap, 20 FUEL/s)", 34, 34, 20, 20, 0.58, 4.39, 2.8, "turret", 0.0, 0.93, passer=True)
ms.ARCH["apex_v2_15"] = A("APEX v2 at 15 FUEL/s", 34, 34, 15, 20, 0.58, 4.39, 2.8, "turret", 0.0, 0.93, passer=True)

SCEN = [
    ("APEX v2 + 2 partners vs 4414-class + 2 partners", ["apex_v2", "partner", "partner"], ["turret4414", "partner", "partner"]),
    ("APEX v2 + 2 partners vs 254-class + 2 partners", ["apex_v2", "partner", "partner"], ["dumper254", "partner", "partner"]),
    ("APEX v2 + 2 partners vs 2910-class + 2 partners", ["apex_v2", "partner", "partner"], ["dumper2910", "partner", "partner"]),
    ("APEX v2 + 2 partners vs 2481-class + 2 partners", ["apex_v2", "partner", "partner"], ["big2481", "partner", "partner"]),
    ("APEX v2 replaces 4414 in a champs alliance", ["apex_v2", "dumper254", "dumper2910"], ["turret4414", "dumper254", "dumper2910"]),
    ("Ablation: 15 FUEL/s feed", ["apex_v2_15", "partner", "partner"], ["dumper254", "partner", "partner"]),
]

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    for name, b, r in SCEN:
        bs, rs, per, ab, ar = ms.play(b, r, n=n)
        print(f"{name:52s} blue {bs:6.1f} | red {rs:6.1f} | APEX/first robot {per[0]:6.1f} vs {per[3]:6.1f}", flush=True)
