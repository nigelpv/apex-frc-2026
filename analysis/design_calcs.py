"""APEX v2 subsystem sizing: drivetrain, intake, serializer, uptake, shooter, hood, turret,
hopper capacity, electrical load and weight. Numbers here match the Onshape CAD (APEX Lib v2) and
robot/src/main/java/frc/robot/Constants.java. Run: python3 design_calcs.py
"""
import math

IN, LB = 0.0254, 0.4536
G = 9.81
FUEL_D_IN = 5.91
FUEL_KG = 0.215


class Motor:
    """DC motor model at 12 V (WPILib DCMotor equivalent)."""

    def __init__(self, name, free_rpm, stall_nm, stall_a, free_a):
        self.name, self.free_rpm, self.stall_nm, self.stall_a, self.free_a = name, free_rpm, stall_nm, stall_a, free_a
        self.kt = stall_nm / stall_a
        self.r = 12.0 / stall_a


X60 = Motor("Kraken X60 FOC", 5800, 9.37, 483, 2.0)
X44 = Motor("Kraken X44 FOC", 7530, 4.11, 279, 1.4)

out = []


def say(s=""):
    out.append(s)
    print(s)


# ---------------------------------------------------------------- frame / mass
FRAME_W, FRAME_L, BUMPER_T = 28.0, 27.0, 3.25
assert 2 * (FRAME_W + FRAME_L) <= 110.0
MASS_KG = 60.3            # CAD, with bumpers and battery
say("== Frame and mass ==")
say(f"Frame {FRAME_W} x {FRAME_L} in (perimeter {2 * (FRAME_W + FRAME_L):.0f} in), "
    f"over bumpers {FRAME_W + 2 * BUMPER_T:.1f} x {FRAME_L + 2 * BUMPER_T:.1f} in")
say(f"Match mass {MASS_KG:.1f} kg ({MASS_KG / LB:.0f} lb) with bumpers and battery")
say("Height 21.75 in with the hood down (TRENCH clear height 22.25 in)")

# ---------------------------------------------------------------- hopper capacity
say("\n== Hopper capacity (estimate from the CAD envelope) ==")
PACK = 0.58               # random packing of spheres against flat walls
V_FUEL = math.pi / 6 * FUEL_D_IN ** 3
box = 13.0 * 26.0 * (21.0 - 4.9)                  # open hopper ahead of the turret deck
bowl = math.pi * 8.1 ** 2 / 2 * (12.75 - 4.9)     # rear half of the rotor bowl under the deck
cap = (box + bowl) * PACK / V_FUEL
say(f"Open hopper {box:.0f} in^3 + rotor bowl under the deck {bowl:.0f} in^3 at {PACK} packing "
    f"-> about {cap:.0f} FUEL (sim uses 32)")

# ---------------------------------------------------------------- drivetrain
say("\n== Drivetrain: SDS MK5n R1 (7.03:1), Kraken X60 FOC drive, X44 steer ==")
WHEEL_D = 4.0 * IN
RATIO = 7.03
v_free = X60.free_rpm / RATIO / 60 * math.pi * WHEEL_D
STATOR = 100.0
f_motor = 4 * X60.kt * STATOR * RATIO / (WHEEL_D / 2) * 0.97
f_trac = MASS_KG * G * 1.1
say(f"Free speed {v_free:.2f} m/s ({v_free / 0.3048:.1f} ft/s), path limit 3.6 m/s")
say(f"Push force at {STATOR:.0f} A stator {f_motor:.0f} N vs traction {f_trac:.0f} N (mu 1.1): "
    f"slip current 100 A keeps the wheels at the traction limit")
a = min(f_motor, f_trac) / MASS_KG
t6 = v_free * 0.92 / a + (6 - (v_free * 0.92) ** 2 / (2 * a)) / (v_free * 0.92)
say(f"Max accel {a:.1f} m/s^2; 0 -> 6 m in {t6:.2f} s")

# ---------------------------------------------------------------- intake
say("\n== Intake: pivoting over-bumper arm, 23 in throat ==")
DEPLOY_RATIO = 25.0 * 36.0 / 12.0
ARM_TRAVEL = 127.0
arm_rate = X44.free_rpm * 0.7 / DEPLOY_RATIO / 60 * 360
say(f"Deploy X44 {DEPLOY_RATIO:.0f}:1 -> {arm_rate:.0f} deg/s, {ARM_TRAVEL:.0f} deg in "
    f"{ARM_TRAVEL / arm_rate:.2f} s (Motion Magic profile 0.4 s)")
ARM_KG, ARM_CG = 2.1, 6.0 * IN
hold = ARM_KG * G * ARM_CG / DEPLOY_RATIO / X44.kt
say(f"Holding the arm horizontal needs {hold:.1f} A; a 15 A stator limit lets it fold on impact")
ROLLER_D = 2.0 * IN
ROLLER_RATIO = 28.0 / 12.0
surf = X44.free_rpm * 11.0 / 12.0 / ROLLER_RATIO / 60 * math.pi * ROLLER_D
say(f"Rollers 2 in at {X44.free_rpm * 11 / 12 / ROLLER_RATIO:.0f} rpm -> surface {surf:.1f} m/s "
    f"(2.3x the 3.6 m/s drive speed, FUEL is never pushed)")
lanes = 23.0 / FUEL_D_IN
say(f"Throat {lanes:.1f} FUEL wide; in the NEUTRAL ZONE staging box (~38 FUEL/m^2) a 0.53 m strip "
    f"gives ~20 FUEL per metre, so the hopper fills in about 1.6 m of sweep")

# ---------------------------------------------------------------- serializer
say("\n== Rotor serializer: 16.2 in rotor, 5 pockets on a 5.2 in circle ==")
ROTOR_RATIO = 160.0 / 12.0
ROTOR_RPS = 4.0
bps = 5 * ROTOR_RPS
say(f"X60 -> 18T:18T -> 12T:160T = {ROTOR_RATIO:.2f}:1; {ROTOR_RPS} rev/s needs "
    f"{ROTOR_RPS * 60 * ROTOR_RATIO:.0f} rpm ({100 * ROTOR_RPS * 60 * ROTOR_RATIO / X60.free_rpm:.0f}% free)")
say(f"Feed rate {bps:.0f} FUEL/s; stall torque at the rotor {X60.stall_nm * ROTOR_RATIO:.0f} N.m "
    f"(80 A stator limit -> {X60.kt * 80 * ROTOR_RATIO:.0f} N.m to break a jam)")
pocket_v = ROTOR_RPS * 2 * math.pi * 5.2 * IN
say(f"Pocket speed {pocket_v:.2f} m/s, centripetal {pocket_v ** 2 / (5.2 * IN) / G:.2f} g")

# ---------------------------------------------------------------- uptake
say("\n== Uptake: twin 15 mm HTD belts on 30T pulleys, X44 direct ==")
PULLEY_PD = 30 * 0.005 / math.pi
belt = X44.free_rpm * 0.85 / 60 * math.pi * PULLEY_PD
say(f"Belt speed at 85% free {belt:.1f} m/s (command 9.0 m/s); FUEL spacing at {bps:.0f}/s: "
    f"{9.0 / bps / IN:.1f} in (> {FUEL_D_IN} in, single file without contact)")

# ---------------------------------------------------------------- shooter
say("\n== Shooter: 4 in flywheel, 2x X60 1:1, fixed hood, exit = 0.47 x surface speed ==")
FLY_D = 4.0 * IN
EXIT_EFF = 0.47
for label, v in (("HUB at 7.0 m", 9.91), ("HUB at 3.0 m", 7.13), ("PASS at 11 m", 11.57)):
    rpm = v / EXIT_EFF / (math.pi * FLY_D) * 60
    say(f"  {label:14s} exit {v:5.2f} m/s -> {rpm:4.0f} rpm ({100 * rpm / X60.free_rpm:.0f}% free)")
I_FLY = 0.0088
w = 3600 / 60 * 2 * math.pi
E = 0.5 * I_FLY * w ** 2
E_ball = 1.3 * 0.5 * FUEL_KG * 8.5 ** 2
dip = 1 - math.sqrt(1 - E_ball / E)
tq = 2 * X60.kt * 80
rec = dip * w * I_FLY / tq
say(f"Flywheel {I_FLY} kg.m^2 stores {E:.0f} J at 3600 rpm; a ball takes {E_ball:.1f} J -> "
    f"{100 * dip:.1f}% dip, recovered in {1000 * rec:.0f} ms at 80 A (balls are {1000 / bps:.0f} ms apart)")
say(f"Spin-up idle 1800 -> 3600 rpm: {I_FLY * (w - w / 2) / tq:.2f} s")

# ---------------------------------------------------------------- hood
say("\n== Hood ==")
HOOD_RATIO = 25.0 * 45.0 / 16.0     # 25:1 MAXPlanetary, 16T:45T 20DP sector
hood_rate = X44.free_rpm * 0.7 / HOOD_RATIO / 60 * 360
say(f"X44 {HOOD_RATIO:.2f}:1 to a 45T sector: {hood_rate:.0f} deg/s, full 25 deg range in "
    f"{25 / hood_rate:.2f} s; launch 45-70 deg; TRENCH guard drives it to the 45 deg stop")

# ---------------------------------------------------------------- turret
say("\n== Turret ==")
TURRET_RATIO = 7.0 * 120.0 / 12.0
slew = X44.free_rpm * 0.8 / TURRET_RATIO / 60 * 360
I_T = 0.035
alpha = X44.kt * 40 * TURRET_RATIO / I_T
say(f"X44 {TURRET_RATIO:.0f}:1 -> {slew:.0f} deg/s (chassis yaw rarely exceeds 360 deg/s); "
    f"{alpha:.0f} rad/s^2 at 40 A; 180 deg in {2 * math.sqrt(math.pi / alpha):.2f} s bang-bang")
rng = 1.0 / abs(120 / 19 - 120 / 21) * 360
say(f"Absolute angle: 19T and 21T CANcoders on the 120T ring -> unique over {rng:.1f} deg "
    f"(service loop sized for +/-190 deg, soft limits +/-190 deg)")

# ---------------------------------------------------------------- electrical
say("\n== Electrical: supply-current budget per match state ==")
R_PATH = 0.020
STATES = {
    "Sweep the NEUTRAL ZONE (drive 4x38, intake 25, rotor 10)": [4 * 38, 4 * 5, 25, 10, 0, 8, 2, 8],
    "Shoot on the move (drive 4x18, flywheel 2x30, rotor+uptake 35)": [4 * 18, 4 * 5, 5, 20, 15, 60, 8, 8],
    "Spin-up while sprinting (drive 4x25, flywheel 2x50)": [4 * 25, 4 * 5, 0, 0, 0, 100, 4, 8],
    "Pushing match (100 A stator at low speed = ~4x30 A supply)": [4 * 30, 4 * 5, 0, 0, 0, 10, 0, 8],
}
for name, parts in STATES.items():
    tot = sum(parts)
    say(f"  {name:66s} {tot:4.0f} A -> bus ~{12.4 - tot * R_PATH:4.1f} V")
say("Supply limits: drive 60 A, flywheel 50 A, rotor 40 A, others 30-35 A; roboRIO 2 brownout 6.75 V")

if __name__ == "__main__":
    with open("design_calcs_output.txt", "w") as f:
        f.write("\n".join(out) + "\n")
