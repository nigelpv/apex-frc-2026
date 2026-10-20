package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * REBUILT (2026) field geometry in WPILib blue-origin coordinates (metres), taken from the 2026
 * welded-field AprilTag layout and the TU22 game manual. Everything is defined for the BLUE
 * alliance; use {@link #flip} for red.
 */
public final class FieldConstants {
  public static final double FIELD_LENGTH = 16.541;
  public static final double FIELD_WIDTH = 8.069;
  public static final double CENTER_X = FIELD_LENGTH / 2.0;

  /** Blue HUB centre and the height of its top opening. */
  public static final Translation2d BLUE_HUB = new Translation2d(4.6255, 4.0345);
  public static final double HUB_OPENING_Z = Units.inchesToMeters(72.0);
  public static final double HUB_OPENING_ACROSS = Units.inchesToMeters(41.7);

  /** Blue ALLIANCE ZONE ends at the HUB's alliance-side face. */
  public static final double ALLIANCE_ZONE_DEPTH = Units.inchesToMeters(158.6);

  /** NEUTRAL ZONE: 283 in deep, centred on the CENTER LINE. */
  public static final double NEUTRAL_ZONE_DEPTH = Units.inchesToMeters(283.0);
  public static final double NEUTRAL_X0 = CENTER_X - NEUTRAL_ZONE_DEPTH / 2.0;

  /** TRENCH lanes along each guardrail (APEX is TRENCH-only: 21.75 in tall vs 22.25 in clear). */
  public static final double TRENCH_LANE_Y_RIGHT = 0.644;
  public static final double TRENCH_LANE_Y_LEFT = 7.425;
  public static final double TRENCH_DEPTH = Units.inchesToMeters(47.0);
  /** The blue-side TRENCH sits between the ALLIANCE ZONE and the NEUTRAL ZONE, at the HUB line. */
  public static final double TRENCH_X = BLUE_HUB.getX();

  /** Passing targets: the corners of our ALLIANCE ZONE, away from the TOWER. */
  public static final Translation2d BLUE_PASS_LEFT = new Translation2d(1.4, 6.6);
  public static final Translation2d BLUE_PASS_RIGHT = new Translation2d(1.4, 1.45);

  /** Neutral-zone FUEL staging box centre. */
  public static final Translation2d NEUTRAL_FUEL_CENTER = new Translation2d(CENTER_X, FIELD_WIDTH / 2);

  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /** Mirror a blue-frame translation into the red frame (the 2026 field is rotationally symmetric). */
  public static Translation2d flip(Translation2d t) {
    return new Translation2d(FIELD_LENGTH - t.getX(), FIELD_WIDTH - t.getY());
  }

  public static Pose2d flip(Pose2d p) {
    return new Pose2d(flip(p.getTranslation()), p.getRotation().rotateBy(Rotation2d.kPi));
  }

  public static Translation2d allianceFlip(Translation2d blue) {
    return isRed() ? flip(blue) : blue;
  }

  public static Pose2d allianceFlip(Pose2d blue) {
    return isRed() ? flip(blue) : blue;
  }

  /** Our HUB. */
  public static Translation2d ourHub() {
    return allianceFlip(BLUE_HUB);
  }

  /** True when the robot is inside our ALLIANCE ZONE (where FUEL may be scored). */
  public static boolean inOurAllianceZone(Translation2d robot) {
    double x = isRed() ? FIELD_LENGTH - robot.getX() : robot.getX();
    return x <= ALLIANCE_ZONE_DEPTH;
  }

  /** Distance (m) along X from the robot into our ALLIANCE ZONE boundary; negative when inside. */
  public static double distanceOutsideAllianceZone(Translation2d robot) {
    double x = isRed() ? FIELD_LENGTH - robot.getX() : robot.getX();
    return x - ALLIANCE_ZONE_DEPTH;
  }

  private FieldConstants() {}
}
