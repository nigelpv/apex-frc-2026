package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide constants for APEX (2026 REBUILT). Geometry comes straight from the Onshape CAD
 * (APEX Lib v2, robot frame: +X toward the intake, +Y left, Z up from the floor). Units are SI
 * unless a name says otherwise.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double LOOP_PERIOD_SEC = 0.02;

  /** CAN IDs on the roboRIO bus (mechanisms). The drivetrain lives on the CANivore, see TunerConstants. */
  public static final class CanIds {
    public static final int INTAKE_ROLLER = 20;
    public static final int INTAKE_DEPLOY = 21;
    public static final int SERIALIZER = 22;
    public static final int UPTAKE = 23;
    public static final int TURRET = 24;
    public static final int TURRET_ENC_19T = 25;
    public static final int TURRET_ENC_21T = 26;
    public static final int FLYWHEEL_LEFT = 27;
    public static final int FLYWHEEL_RIGHT = 28;
    public static final int HOOD = 29;
    public static final int CANRANGE_UPTAKE = 30;
    public static final int CANRANGE_HOPPER = 31;
    public static final int PDH = 1;

    private CanIds() {}
  }

  /** Over-bumper intake: arm on a 1/2 hex pivot, three 2 in silicone rollers. */
  public static final class IntakeConstants {
    /** X44 -> MAXPlanetary 5:1 + 5:1 -> 12T:36T belt to the pivot. */
    public static final double DEPLOY_RATIO = 25.0 * 36.0 / 12.0;
    /** Arm angle measured from deployed (0) up to stowed (127 deg, arm vertical). */
    public static final double STOWED_RAD = Units.degreesToRadians(127.0);
    public static final double DEPLOYED_RAD = 0.0;
    /** Hard-stop tolerance band where the arm counts as at a position. */
    public static final double AT_POSITION_TOL_RAD = Units.degreesToRadians(4.0);
    /** X44 -> 12T:28T to the rear top roller, 1:1 to the others. */
    public static final double ROLLER_RATIO = 28.0 / 12.0;
    public static final double ROLLER_DIA_M = Units.inchesToMeters(2.0);
    public static final double INTAKE_VOLTS = 11.0;
    public static final double EJECT_VOLTS = -8.0;
    /** Stator limit while deployed: the arm gives way on impact instead of bending. */
    public static final double DEPLOY_HOLD_STATOR_A = 15.0;
    public static final double ARM_MOI = 0.35; // kg m^2 about the pivot (CAD mass properties)
    public static final double ARM_LENGTH_M = Units.inchesToMeters(10.5);

    private IntakeConstants() {}
  }

  /** Rotor serializer (160T ring gear, five pockets) and the twin-belt uptake. */
  public static final class IndexerConstants {
    /** X60 -> 18T:18T belt -> 12T pinion on the 160T rotor gear. */
    public static final double ROTOR_RATIO = 160.0 / 12.0;
    public static final int ROTOR_POCKETS = 5;
    /** Uptake: X44 direct on the upper 30T HTD pulley shaft. */
    public static final double UPTAKE_PULLEY_PITCH_DIA_M = 30 * 0.005 / Math.PI;
    public static final double UPTAKE_RATIO = 1.0;
    /**
     * Rotor feed speed while shooting (rotor rev/s). 5 pockets x 4 rev/s = 20 FUEL/s; the X60 runs
     * at 55 % of free speed and the flywheel recovers in 9 ms of the 50 ms between balls. Drop to
     * 3.0 if pockets start arriving empty when the hopper is low.
     */
    public static final double ROTOR_FEED_RPS = 4.0;
    public static final double ROTOR_AGITATE_RPS = 0.6;
    public static final double UPTAKE_FEED_MPS = 9.0;
    /** CANrange distance below which a FUEL is present at the sensor (m). */
    public static final double UPTAKE_BALL_DIST_M = 0.07;
    public static final double HOPPER_FULL_DIST_M = 0.20;

    private IndexerConstants() {}
  }

  /** Turret: Kaydon KC090XP0 ring bearing, 120T 10DP plate, X44 + 7:1 to a 12T pinion. */
  public static final class TurretConstants {
    public static final double MOTOR_TO_TURRET = 7.0 * 120.0 / 12.0; // 70:1
    public static final int RING_TEETH = 120;
    public static final int ENC_A_TEETH = 19;
    public static final int ENC_B_TEETH = 21;
    /**
     * Soft limits. The turret harness is a horizontal service loop between the fixed wrap ring
     * and the rotating drum; it is sized for +/-190 deg with 10 deg of loop margin at each end.
     */
    public static final double MIN_RAD = Units.degreesToRadians(-190.0);
    public static final double MAX_RAD = Units.degreesToRadians(190.0);
    /** Turret axis in the robot frame (m). */
    public static final Translation2d AXIS_IN_ROBOT =
        new Translation2d(Units.inchesToMeters(-4.5), 0.0);
    public static final double MOI = 0.035; // kg m^2 (CAD)
    public static final double MAX_VEL_RAD_PER_SEC = Units.degreesToRadians(500.0);
    public static final double MAX_ACCEL_RAD_PER_SEC2 = Units.degreesToRadians(4000.0);
    public static final double AIM_TOL_RAD = Units.degreesToRadians(1.5);

    private TurretConstants() {}
  }

  /** Shooter: three 4 in 60A wheels + steel discs, 2x X60 1:1; hood on a 45T sector. */
  public static final class ShooterConstants {
    public static final double FLYWHEEL_RATIO = 1.0;
    public static final double WHEEL_DIA_M = Units.inchesToMeters(4.0);
    /** Ball exit speed / wheel surface speed (measured on the prototype, update after tuning). */
    public static final double EXIT_EFFICIENCY = 0.47;
    public static final double FLYWHEEL_MOI = 0.0088; // kg m^2 (wheels + discs)
    public static final double MAX_RPM = 5600.0;
    public static final double RPM_TOL = 60.0;
    /** X44 -> MAXPlanetary 5:1 + 5:1 -> 16T:45T 20DP sector (70.3:1). */
    public static final double HOOD_RATIO = 25.0 * 45.0 / 16.0;
    /** Launch angle at hood lift 0 (fully down, TRENCH-safe). */
    public static final double MIN_LAUNCH_DEG = 45.0;
    public static final double MAX_LAUNCH_DEG = 70.0;
    public static final double HOOD_TOL_DEG = 0.75;
    /** Release point height above the floor (flywheel contact patch). */
    public static final double RELEASE_HEIGHT_M = Units.inchesToMeters(18.2);

    private ShooterConstants() {}
  }

  /** Whole-robot numbers used by sim, path planning and the power manager. */
  public static final class RobotConstants {
    public static final double MASS_KG = 60.3; // 133 lb with bumpers and battery (CAD + COTS)
    public static final double MOI_KG_M2 = 5.9;
    public static final double MAX_HEIGHT_M = Units.inchesToMeters(21.75); // TRENCH clear 22.25 in
    public static final double FRAME_LENGTH_M = Units.inchesToMeters(27.0);
    public static final double FRAME_WIDTH_M = Units.inchesToMeters(28.0);
    public static final double BUMPER_DEPTH_M = Units.inchesToMeters(3.25);
    /** Bus voltage floor the power manager defends (roboRIO 2 brownout is 6.75 V). */
    public static final double BROWNOUT_GUARD_V = 7.4;

    private RobotConstants() {}
  }

  private Constants() {}
}
