package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.List;

/**
 * AUTO routines, built in code as on-the-fly PathPlanner paths (blue-alliance coordinates;
 * AutoBuilder mirrors them for red).
 *
 * <p>The superstructure does all the scoring by itself: it fires whenever the BUMPERS are in our
 * ALLIANCE ZONE, the turret is on target and the robot is clear of the TRENCH. The routines only
 * drive and switch the intake.
 *
 * <p>"Double Sweep": fire the 8 preloaded FUEL from the starting line, drive under the TRENCH,
 * sweep a 2.4 m strip of the NEUTRAL ZONE staging box (about 20 FUEL per metre, so the hopper is
 * full by the end of the strip), come back under the other TRENCH firing as soon as we are in the
 * ALLIANCE ZONE, then do it again on the other half of the box. Every pose keeps the BUMPERS on
 * our side of the CENTER LINE (G403). Estimated timeline: preload 1.5 s, out 2.4 s, sweep 1.6 s,
 * back 2.7 s, unload 2.0 s, out 2.2 s, sweep 1.5 s, back 2.6 s, unload until the end of AUTO.
 */
public final class Autos {
  private static final PathConstraints FAST =
      new PathConstraints(3.6, 5.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  private static final PathConstraints SWEEP =
      new PathConstraints(1.6, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(720));

  /** Starting pose: BUMPERS overlap the ROBOT STARTING LINE in front of the right BUMP (G303). */
  public static final Pose2d RIGHT_START = new Pose2d(3.60, 2.45, Rotation2d.kZero);

  private static final double LANE_Y = FieldConstants.TRENCH_LANE_Y_RIGHT + 0.03;
  private static final double SWEEP_X = 7.55; // bumpers end 0.28 m short of the CENTER LINE

  private Autos() {}

  /** Stand still and fire the preload. */
  public static Command shootPreload(Drive drive, Superstructure ss, Pose2d start) {
    return Commands.sequence(
            AutoBuilder.resetOdom(start),
            Commands.runOnce(() -> ss.setHoldingFuel(true)),
            Commands.waitUntil(() -> !ss.isHoldingFuel()).withTimeout(3.0))
        .withName("Shoot Preload");
  }

  /**
   * The main AUTO.
   *
   * @param left true for the mirror-image routine that starts in front of the left BUMP
   */
  public static Command doubleSweep(Drive drive, Superstructure ss, boolean left) {
    Pose2d start = m(RIGHT_START, left);

    // The robot only turns outside the TRENCH: under it the heading is held at a multiple of
    // 90 deg (turned 45 deg the bumpers would need 1.22 m of the 1.28 m lane).

    // out under the right TRENCH (heading 0), then turn so the intake faces +Y
    PathPlannerPath out1a =
        path(
            FAST,
            2.5,
            0,
            left,
            pose(3.60, 2.45, -115),
            pose(3.10, LANE_Y + 0.1, -10),
            pose(6.00, LANE_Y, 0));
    PathPlannerPath out1b = path(FAST, 1.5, 90, left, pose(6.00, LANE_Y, 0), pose(SWEEP_X, 1.55, 90));
    PathPlannerPath sweep1 = path(SWEEP, 1.0, 90, left, pose(SWEEP_X, 1.55, 90), pose(SWEEP_X, 3.95, 90));
    // back under the left TRENCH into the ALLIANCE ZONE, heading held at 90
    PathPlannerPath back1 =
        path(
            FAST,
            0.0,
            90,
            left,
            pose(SWEEP_X, 3.95, 90),
            pose(6.60, 6.85, 135),
            pose(5.60, 7.40, 180),
            pose(3.20, 7.40, 180),
            pose(2.40, 6.30, -120));
    // out again under the left TRENCH (heading 90), then turn so the intake faces -Y
    PathPlannerPath out2a =
        path(
            FAST,
            2.5,
            90,
            left,
            pose(2.40, 6.30, 60),
            pose(3.20, 7.40, 0),
            pose(6.00, 7.40, 0));
    PathPlannerPath out2b = path(FAST, 1.5, -90, left, pose(6.00, 7.40, 0), pose(SWEEP_X, 6.55, -90));
    PathPlannerPath sweep2 = path(SWEEP, 0.0, -90, left, pose(SWEEP_X, 6.55, -90), pose(SWEEP_X, 4.20, -90));
    PathPlannerPath back2 =
        path(
            FAST,
            0.0,
            -90,
            left,
            pose(SWEEP_X, 4.20, 110),
            pose(6.60, 6.85, 135),
            pose(5.60, 7.40, 180),
            pose(3.20, 7.40, 180),
            pose(2.40, 6.30, -120));

    return Commands.sequence(
            AutoBuilder.resetOdom(start),
            Commands.runOnce(() -> ss.setHoldingFuel(true)),
            Commands.waitUntil(() -> !ss.isHoldingFuel()).withTimeout(1.6),
            // trip 1 (the intake deploys once the robot is clear of the TRENCH)
            AutoBuilder.followPath(out1a),
            AutoBuilder.followPath(out1b).deadlineFor(intakeOn(ss)),
            AutoBuilder.followPath(sweep1).deadlineFor(intakeOn(ss)),
            Commands.runOnce(() -> ss.setWantIntake(false)),
            AutoBuilder.followPath(back1),
            Commands.waitUntil(() -> !ss.isHoldingFuel()).withTimeout(2.0),
            // trip 2
            AutoBuilder.followPath(out2a),
            AutoBuilder.followPath(out2b).deadlineFor(intakeOn(ss)),
            AutoBuilder.followPath(sweep2).deadlineFor(intakeOn(ss)),
            Commands.runOnce(() -> ss.setWantIntake(false)),
            AutoBuilder.followPath(back2),
            Commands.waitUntil(() -> !ss.isHoldingFuel()))
        .finallyDo(() -> ss.setWantIntake(false))
        .withName(left ? "Left Double Sweep" : "Right Double Sweep");
  }

  /** Teleop helper: drive back into our ALLIANCE ZONE through the nearer TRENCH. */
  public static Command returnThroughTrench(Drive drive) {
    return Commands.defer(
        () -> {
          boolean upper = blueY(drive) > FieldConstants.FIELD_WIDTH / 2;
          Pose2d target = upper ? new Pose2d(2.6, 6.3, Rotation2d.kZero) : new Pose2d(2.6, 1.75, Rotation2d.kZero);
          return AutoBuilder.pathfindToPoseFlipped(target, FAST, 0.0);
        },
        java.util.Set.of(drive));
  }

  private static Command intakeOn(Superstructure ss) {
    return Commands.startEnd(() -> ss.setWantIntake(true), () -> {});
  }

  private static double blueY(Drive drive) {
    double y = drive.getPose().getY();
    return FieldConstants.isRed() ? FieldConstants.FIELD_WIDTH - y : y;
  }

  private static Pose2d pose(double x, double y, double headingDeg) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
  }

  /** Mirror a right-side blue pose to the left side (across the long centre line of the field). */
  private static Pose2d m(Pose2d p, boolean left) {
    return left
        ? new Pose2d(p.getX(), FieldConstants.FIELD_WIDTH - p.getY(), p.getRotation().unaryMinus())
        : p;
  }

  private static PathPlannerPath path(
      PathConstraints c, double endVel, double endHeadingDeg, boolean left, Pose2d... poses) {
    Pose2d[] wp = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      wp[i] = m(poses[i], left);
    }
    Rotation2d end = Rotation2d.fromDegrees(left ? -endHeadingDeg : endHeadingDeg);
    return new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(List.of(wp)), c, null, new GoalEndState(endVel, end));
  }
}
