"""PathPlanner navgrid for APEX (TRENCH-only robot: BUMPS are walls for us).

Obstacles (blue side, mirrored to red): HUB, BUMP + TRENCH legs between the HUB and each TRENCH
lane, TOWER, field perimeter. Everything is inflated by the robot's half width with bumpers so the
pathfinder can treat the robot as a point. The TRENCH lanes stay open (lane centre 0.644 m from the
guardrail). Writes robot/src/main/deploy/pathplanner/navgrid.json.
"""
import json, math, os
from field import FIELD_LENGTH as FL, FIELD_WIDTH as FW, BLUE_HUB, IN

NODE = 0.30
INFLATE = 0.44            # (28 in frame + 2 x 3.25 in bumpers) / 2 = 0.438 m
HUB_HALF = 47.0 * IN / 2
LANE_W = 50.34 * IN       # clear width under the TRENCH arm
BUMP_DEPTH = 47.0 * IN    # TRENCH depth (bump is 44.4 in, trench legs 47 in)
TOWER_W, TOWER_D = 49.25 * IN, 45.18 * IN
TOWER_Y = (3.746 + 4.178) / 2

hx, hy = BLUE_HUB
rects = [
    (hx - HUB_HALF, hx + HUB_HALF, hy - HUB_HALF, hy + HUB_HALF),                 # HUB
    (hx - BUMP_DEPTH / 2, hx + BUMP_DEPTH / 2, LANE_W, hy - HUB_HALF),           # right BUMP + leg
    (hx - BUMP_DEPTH / 2, hx + BUMP_DEPTH / 2, hy + HUB_HALF, FW - LANE_W),      # left BUMP + leg
    (0.0, TOWER_D, TOWER_Y - TOWER_W / 2, TOWER_Y + TOWER_W / 2),                # TOWER
]
rects += [(FL - x1, FL - x0, FW - y1, FW - y0) for (x0, x1, y0, y1) in rects]


def blocked(x, y):
    if x < INFLATE or y < INFLATE or x > FL - INFLATE or y > FW - INFLATE:
        return True
    for x0, x1, y0, y1 in rects:
        if x0 - INFLATE < x < x1 + INFLATE and y0 - INFLATE < y < y1 + INFLATE:
            return True
    return False


nx, ny = math.ceil(FL / NODE), math.ceil(FW / NODE)
grid = [[blocked((i + 0.5) * NODE, (j + 0.5) * NODE) for i in range(nx)] for j in range(ny)]
out = os.path.join(os.path.dirname(__file__), "..", "robot", "src", "main", "deploy", "pathplanner", "navgrid.json")
json.dump({"field_size": {"x": FL, "y": FW}, "nodeSizeMeters": NODE, "grid": grid}, open(out, "w"))
for j in reversed(range(ny)):
    print("".join("#" if grid[j][i] else "." for i in range(nx)))
