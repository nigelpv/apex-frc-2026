"""REBUILT 2026 field geometry (blue-origin WPILib coordinates, metres).

Derived from the official 2026 welded-field AprilTag layout (WPILib v2026.2.2)
and the TU22 game manual. Blue alliance wall is at x = 0.
"""
import json, math, os

IN = 0.0254

FIELD_LENGTH = 16.541
FIELD_WIDTH = 8.069
CENTER_X = FIELD_LENGTH / 2

# HUB (blue). Tag faces at x = 4.022 / 5.229, y = 3.431 / 4.638 (manual: 47 in square)
BLUE_HUB = (4.6255, 4.0345)
RED_HUB = (FIELD_LENGTH - 4.6255, 4.0345)
HUB_SIZE = 47.0 * IN                 # 1.194 m footprint
HUB_OPENING_Z = 72.0 * IN            # front edge of top opening, 1.829 m
HUB_OPENING_ACROSS = 41.7 * IN       # hexagonal opening, 1.059 m

# ALLIANCE ZONE: 158.6 in from the wall -> ends at the hub's alliance-side face
ALLIANCE_ZONE_DEPTH = 158.6 * IN     # 4.028 m
# NEUTRAL ZONE: 283 in deep, centred on the CENTER LINE
NEUTRAL_ZONE_DEPTH = 283.0 * IN
NEUTRAL_X0 = CENTER_X - NEUTRAL_ZONE_DEPTH / 2   # 4.676 m (blue side boundary)

# TRENCH: 22.25 in clear height, 50.34 in clear width, 47 in deep, along each guardrail
TRENCH_CLEAR_H = 22.25 * IN
TRENCH_CLEAR_W = 50.34 * IN
TRENCH_DEPTH = 47.0 * IN
TRENCH_LANES_Y = (0.644, 7.425)      # trench AprilTag y (approx lane centres)

# BUMP: 73 in wide, 44.4 in deep, 6.513 in tall, 15 deg HDPE ramps (peaked)
BUMP_W, BUMP_D, BUMP_H = 73.0 * IN, 44.4 * IN, 6.513 * IN
BUMP_ANGLE = math.radians(15)

# TOWER (blue) on the alliance wall, rungs at 27 / 45 / 63 in
TOWER_Y = (3.746 + 4.178) / 2
RUNG_Z = (27.0 * IN, 45.0 * IN, 63.0 * IN)

# Neutral-zone FUEL staging box: 206 in (field width) x 72 in (field length)
NZ_BOX = (CENTER_X - 36 * IN, CENTER_X + 36 * IN,
          FIELD_WIDTH / 2 - 103 * IN, FIELD_WIDTH / 2 + 103 * IN)

FUEL_D = 5.91 * IN                   # 0.150 m
FUEL_MASS = (0.448 + 0.500) / 2 * 0.4536   # 0.215 kg

# Robot envelope rules
MAX_PERIMETER = 110 * IN
MAX_HEIGHT = 30.0 * IN
MAX_EXTENSION = 12.0 * IN
BUMPER_ZONE = (2.5 * IN, 5.75 * IN)
MAX_WEIGHT_LB = 115.0


def load_tags():
    path = os.path.join(os.path.dirname(__file__), "2026-rebuilt-welded.json")
    with open(path) as f:
        return json.load(f)["tags"]


if __name__ == "__main__":
    print(f"Blue hub centre {BLUE_HUB}, opening z {HUB_OPENING_Z:.3f} m")
    print(f"Alliance zone depth {ALLIANCE_ZONE_DEPTH:.3f} m; neutral zone starts x={NEUTRAL_X0:.3f}")
    d_far = math.hypot(BLUE_HUB[0], BLUE_HUB[1])
    print(f"Farthest legal shot (alliance corner): {d_far:.2f} m")
    print(f"Bump peak run per ramp: {BUMP_D/2:.3f} m; peak-crossing clearance needed for "
          f"0.55 m wheelbase: {0.275*math.tan(BUMP_ANGLE)/IN:.2f} in")
