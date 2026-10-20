# APEX design notes

Numbers come from `analysis/` (run the scripts to regenerate) and match the Onshape CAD and
`Constants.java`.

## Game facts the design is built on (2026 manual, TU22)

- AUTO 20 s, TELEOP 140 s: TRANSITION (both HUBS active) 140-130, four 25 s ALLIANCE SHIFTS,
  END GAME (both active) 30-0. The alliance that scored more FUEL in AUTO has its HUB inactive in
  SHIFT 1 and 3; FMS game data says which alliance that was. FUEL still counts for 3 s after a
  HUB turns off.
- G407: FUEL may only be launched into the HUB while the robot's BUMPERS are at least partly in
  its ALLIANCE ZONE. Deliberately scoring a lot from the NEUTRAL ZONE is a card offence.
- G403: in AUTO a robot may not get its BUMPERS completely across the CENTER LINE.
- TRENCH clear height 22.25 in, 50.34 in wide; BUMPS 6.5 in tall with 15 deg ramps.
- 504 FUEL per match: 24 per DEPOT, 24 per OUTPOST chute, up to 8 preloaded per robot, the rest
  (360+) staged in the NEUTRAL ZONE.

## What wins REBUILT (from the match simulator)

`analysis/match_sim.py` tracks every FUEL through a full match with six robots following simple
policies, HUB shifts, scoring delays and FUEL recycling out of the HUB exits. Reference robots
use published 2026 specs of Einstein-level teams (4-wide dumpers, a turret robot, a 115-ball
robot). `analysis/match_sim_v2.py` runs APEX v2 as it is built (34 FUEL hopper, 20 FUEL/s turret
feed); results are in `analysis/match_sim_v2_output.txt`.

| Scenario (6 matches each) | APEX v2 alone | Reference robot alone | Alliance score |
| --- | --- | --- | --- |
| APEX + 2 average partners vs 4414-class turret + 2 partners | 584 | 247 | 915 vs 518 |
| APEX + 2 partners vs 254-class 4-wide dumper + 2 partners | 564 | 247 | 903 vs 534 |
| APEX + 2 partners vs 2910-class high-rate dumper + 2 partners | 566 | 216 | 899 vs 506 |
| APEX + 2 partners vs 2481-class 115-ball robot + 2 partners | 592 | 290 | 917 vs 578 |
| APEX replaces the 4414-class robot in a champs-style alliance | 524 | 223 | 1026 vs 635 |
| Ablation: the same robot with a 15 FUEL/s feed vs 254-class | 279 | 254 | 575 vs 555 |

Treat the absolute numbers with care: the policies are simple and the model ignores defense and
robot collisions. The ranking and the ablations are the useful part.

The model's main lessons:

1. **Firing rate and no alignment stop matter most.** A robot that can fire while driving, from
   anywhere in the zone, turns every second in the zone into scoring time.
2. **Cycle time through the TRENCH beats raw capacity.** A short robot that never climbs the BUMP
   and carries ~35 FUEL cycles faster than a 100-ball robot that has to.
3. **Use the inactive SHIFTS.** While our HUB is off the robot collects in the NEUTRAL ZONE and can
   lob FUEL into our zone (auto-pass), so it starts the next active SHIFT loaded with a stockpile.

## Subsystems

| Subsystem | Design | Key numbers |
| --- | --- | --- |
| Drivetrain | SDS MK5n R1, X60 drive / X44 steer, CANivore + Pigeon 2 | 4.39 m/s free, 10.8 m/s^2, 28 x 27 in frame |
| Intake | Pivoting over-bumper arm, 2 in silicone rollers, 23 in throat | X44 75:1 deploy (0.3 s), compliant 15 A hold, rollers 7.9 m/s |
| Serializer | 16.2 in rotor ("dye rotor"), 5 pockets, X60 13.33:1 | 4 rev/s = 20 FUEL/s single file |
| Uptake | Twin 15 mm HTD belts through the turret bearing, X44 direct | 9 m/s belt, 24 in ball spacing |
| Turret | Kaydon KC090XP0 ring bearing, 120T plate, X44 70:1, horizontal service-loop cable wrap | +/-190 deg soft limits, 516 deg/s, vernier 19T/21T CANcoders (598 deg unique) |
| Shooter | 4 in flywheel, 2x X60 1:1 (18T:18T belts), hood 45-70 deg on a 45T sector (X44 70.3:1) | 2400-4000 rpm for HUB shots, 0.8 % dip per ball, 9 ms recovery |
| Vision | Limelight 4 on the turret + two fixed rear Limelight 4s, MegaTag 2 | turret camera pose pushed every loop from the turret angle |
| Electrical | REV PDH, roboRIO 2, CANivore for the drive, VH-109 radio, MPM, 120 A breaker | budget in `design_calcs_output.txt` |

## Turret cable wrap

The turret harness (three motor power pairs, CAN and Cat6, zip-tied to a polycarb backing strip,
0.6 x 0.3 in) is a horizontal service loop in an annular channel above the turret gears: a fixed
printed ring (floor and wall, on six posts from the deck) outside, a rotating printed drum inside.
From its fixed end on the ring wall the loop runs around the wall, makes a U-turn and runs back
along the drum to its moving end. The U-turn moves at 0.46x the turret angle, so the loop covers
+/-190 deg with 10 deg of margin at each end, and neither run ever wraps more than 217 deg. The
Onshape model redraws the loop for whatever turret angle is configured.

Every gear that meshes with the 120T turret plate (yaw pinion, both vernier gears) sits on a plate
tooth gap and is phased for the configured turret angle, and the hood pinion is phased for the
configured hood lift, so the gears mesh cleanly in every configuration.

## Shooting

`analysis/ballistics.py` models FUEL as a 5.91 in, 0.215 kg sphere with quadratic drag
(Cd 0.47) and backspin Magnus lift, integrated with RK4. For each distance and hood angle it
solves the exit speed that drops the ball through the centre of the HUB opening with rim
clearance, then scores the shot with 150 perturbed launches (2 % speed, 0.5 deg angle, mass, drag
and spin spread). `ballistics_v2_smooth.py` picks the angle curve with dynamic programming so the
hood angle never rises with distance; interpolated shots stay on a tested trajectory family.
Make probability: 0.9 to 0.99 out to 4.5 m, 0.8 to 0.87 out to 7 m (the farthest point in the
ALLIANCE ZONE is about 5.5 m from the HUB).

The robot aims at a virtual target (`ShotSolver`): the HUB minus robot velocity times time of
flight, iterated to convergence, with turret rate feed-forward. A unit test flies every solution
through the same physics and checks the ball lands inside the opening for robot speeds up to
3 m/s and spins up to 2 rad/s.

## Fire control (Superstructure)

Fires when all of these hold:

- BUMPERS at least 10 cm inside our ALLIANCE ZONE (G407 with margin for pose error),
- the shot will count: our HUB is active when the ball arrives, or went inactive less than 3 s
  before (AUTO and missing game data count as always active),
- turret on target, flywheel within 60 rpm, hood within 0.75 deg, distance inside the table,
  chassis yaw rate under 4 rad/s,
- not within 0.4 s of a TRENCH (the hood is also forced down there).

Near a TRENCH the drive command also squares the heading to the nearest 90 deg whenever the
driver is not turning: turned 45 deg the bumpers would need 1.22 m of the 1.28 m lane.

The flywheel spins up early: when our HUB will be active within 0.6 s plus the time of flight,
and the robot is within 1.5 m of the zone. Between shots it idles at 1800 rpm.

## AUTO

"Double Sweep" (left or right): fire the preload from the starting line, run under the TRENCH,
sweep a 2.4 m strip of the NEUTRAL ZONE staging box (the hopper fills in about 1.6 m), come back
under the other TRENCH firing as soon as the BUMPERS are in the zone, then repeat on the other
half of the box. All poses stay on our side of the CENTER LINE.

## Known limits and next steps

- Hopper capacity is an envelope estimate (~34 FUEL). Weigh a full hopper on the prototype.
- The turret service loop bends the 12 AWG silicone pairs at a 0.7 in radius. Cycle-test a
  sample loop (a few thousand +/-190 deg sweeps) before trusting it in matches.
- `EXIT_EFFICIENCY` (0.47) and the shot tables need on-field tuning; the operator has a +/-5 %
  speed trim.
- The DEPOT/OUTPOST AUTO is not written yet (needs exact DEPOT coordinates from the field
  drawings).
