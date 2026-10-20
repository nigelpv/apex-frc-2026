# APEX Lib v2 (FeatureScript source)

The Onshape Feature Studio "APEX Robot Source" is these files concatenated in this order:

1. `00_header.fs` - FeatureScript version, imports of the vendor COTS models (SDS MK5n, Kraken
   X60/X44, Limelight 4, CANrange, roboRIO 2, PDH, MPM, RSL, CANivore, battery, breaker, VH-109,
   switch, Pigeon 2)
2. `01_kernel.fs` - shared constants (robot frame in inches: +X toward the intake, +Y left, Z up
   from the floor) and helpers: tubes, plates, bolts, cables, naming, appearances
3. `02_drivetrain.fs` - "APEX 01 Drivetrain": frame, bellypan, MK5n modules
4. `03_bumpers.fs` - "APEX 02 Bumpers": backing, pool noodles, fabric, numbers, clips
5. `04_serializer.fs` - "APEX 03 Serializer": rotor, bowl, drive pod
6. `05_hopper.fs` - "APEX 04 Hopper + Uptake": uprights, walls, turret deck, funnel, uptake
7. `06_turret.fs` - "APEX 05 Turret": bearing, plate, yaw drive, flywheel, hood, cable-wrap ring, drum and service loop
8. `07_intake.fs` - "APEX 06 Intake": arm, rollers, deploy drive
9. `08_electrical.fs` - "APEX 07 Electrical": battery, breaker, PDH, roboRIO, CANivore, MPM,
   radio, switch, RSL, Pigeon, cameras, CANranges and their mounts
10. `09_wiring.fs` - "APEX 08 Wiring": power, motor harnesses, CAN, network, turret harness to the cable wrap

The Part Studio "APEX Robot" (the full robot) has one feature per file 3-10. Its configuration
panel switches the intake (Deployed / Stowed), the turret angle (-190 to 190 deg), the hood lift
(0 to 25 deg, 0 = 45 deg launch) and the alliance colour of the bumpers.
