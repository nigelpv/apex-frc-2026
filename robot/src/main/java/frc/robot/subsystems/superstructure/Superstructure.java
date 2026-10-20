package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.FieldConstants;
import frc.robot.lib.HubShift;
import frc.robot.lib.ShotSolver;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's brain. The driver drives and holds "intake"; everything else is automatic:
 *
 * <ul>
 *   <li>the turret tracks our HUB every loop (or the nearest pass corner when passing), with
 *       shoot-on-the-move lead from {@link ShotSolver};
 *   <li>the flywheel spins up early enough to be ready when our HUB turns active (HUB shift
 *       schedule from the FMS game data) and when the robot is about to enter our ALLIANCE ZONE;
 *   <li>FUEL is fed whenever the shot is legal (G407: BUMPERS in our ALLIANCE ZONE), will count
 *       (HUB active at arrival, or inside the 3 s grace), the turret is on target and the wheel and
 *       hood are at speed;
 *   <li>near a TRENCH the hood is forced down and feeding stops so the robot fits under 22.25 in.
 * </ul>
 *
 * Operator overrides: force-shoot (ignores the HUB schedule, still obeys G407), pass, eject, hold
 * fire, and auto-pass (lob everything we pick up in the NEUTRAL ZONE back into our ALLIANCE ZONE).
 */
public class Superstructure extends SubsystemBase {
  public enum Target {
    NONE,
    HUB,
    PASS
  }

  private static final double SPINUP_LEAD_SEC = 0.6;
  private static final double EMPTY_TIMEOUT_SEC = 0.6;
  private static final double PRESPIN_DISTANCE_M = 1.5;
  private static final double ZONE_MARGIN_M = 0.10;
  private static final double MAX_FIRE_OMEGA = 4.0; // rad/s, turret feedforward lag above this
  private static final double TRENCH_LOOKAHEAD_SEC = 0.4;
  /** Robot centre to bumper face along X, plus clearance for pose error. */
  private static final double TRENCH_HALF_LENGTH_M =
      RobotConstants.FRAME_LENGTH_M / 2 + RobotConstants.BUMPER_DEPTH_M + 0.15;
  /** A robot whose centre is closer than this to a side wall is in a TRENCH lane. */
  private static final double TRENCH_LANE_EDGE_M = 1.29 + RobotConstants.FRAME_WIDTH_M / 2;

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final ShotSolver hubSolver;
  private final ShotSolver passSolver;
  private final Supplier<HubShift.AutoWinner> autoWinner;

  // requests
  private boolean wantIntake = false;
  private boolean wantForceShoot = false;
  private boolean wantPass = false;
  private boolean wantEject = false;
  private boolean holdFire = false;
  private boolean autoPass = false;
  private double speedTrim = 0.0; // fraction added to every requested exit speed

  // state
  private boolean holdingFuel = true; // the robot starts preloaded (up to 8 FUEL)
  private double lastBallSeenTime = 0.0;
  private Target target = Target.NONE;
  private ShotSolver.Solution solution = null;
  private boolean firing = false;

  public Superstructure(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret,
      Shooter shooter,
      ShotSolver hubSolver,
      ShotSolver passSolver,
      Supplier<HubShift.AutoWinner> autoWinner) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.turret = turret;
    this.shooter = shooter;
    this.hubSolver = hubSolver;
    this.passSolver = passSolver;
    this.autoWinner = autoWinner;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    Pose2d pose = drive.getPose();
    ChassisSpeeds v = drive.getFieldVelocity();

    // ---------------- FUEL bookkeeping
    if (indexer.isBallStaged() || indexer.isHopperFull()) {
      lastBallSeenTime = now;
      holdingFuel = true;
    }
    if (wantIntake && intake.isDeployed() && intake.rollerSurfaceSpeed() > 3.0) {
      // the hopper sensor only sees a nearly full hopper, so assume a pickup while intaking;
      // the feed timeout below clears the flag when nothing comes up
      holdingFuel = true;
      lastBallSeenTime = now;
    }

    // ---------------- target choice
    double outside = zoneDepthOutside(pose);
    boolean inZone = outside < -ZONE_MARGIN_M;
    if (wantPass || (autoPass && !inZone && holdingFuel && !wantForceShoot)) {
      target = Target.PASS;
    } else if (holdingFuel || wantForceShoot) {
      target = Target.HUB;
    } else {
      target = Target.NONE;
    }

    // ---------------- solve + aim (the turret pre-aims at the HUB even when there is nothing to shoot)
    Translation2d aimPoint =
        target == Target.PASS ? nearestPassTarget(pose.getTranslation()) : FieldConstants.ourHub();
    ShotSolver solver = target == Target.PASS ? passSolver : hubSolver;
    solution =
        solver.solve(
            new ShotSolver.Input(
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians(),
                v.vxMetersPerSecond,
                v.vyMetersPerSecond,
                v.omegaRadiansPerSecond,
                aimPoint.getX(),
                aimPoint.getY()));
    if (DriverStation.isEnabled()) {
      turret.aim(solution.turretRobotAngle(), solution.turretRobotRate());
    } else {
      turret.stop();
    }

    // ---------------- TRENCH guard (hood down, no feeding)
    boolean trench = nearTrench(pose, v);
    shooter.setTrenchGuard(trench);

    // ---------------- HUB schedule
    boolean auto = DriverStation.isAutonomous();
    double matchTime = DriverStation.getMatchTime();
    HubShift.AutoWinner winner = autoWinner.get();
    boolean timed = !auto && matchTime >= 0;
    boolean hubCounts = !timed || HubShift.shotWillCount(matchTime, solution.tofSec(), winner);
    boolean hubSoon =
        !timed
            || HubShift.ourHubActive(matchTime, winner)
            || HubShift.secondsToNextChange(matchTime, winner) < SPINUP_LEAD_SEC + solution.tofSec();

    // ---------------- shooter
    boolean spin =
        switch (target) {
          case PASS -> true;
          case HUB -> wantForceShoot || (hubSoon && outside < PRESPIN_DISTANCE_M);
          case NONE -> false;
        };
    if (spin) {
      shooter.setShot(solution.exitSpeedMps() * (1.0 + speedTrim), solution.launchDeg());
    } else if (holdingFuel) {
      shooter.setIdle();
    } else {
      shooter.stop();
    }

    // ---------------- fire decision
    boolean legal = target == Target.PASS || (target == Target.HUB && inZone);
    boolean worth = target == Target.PASS || wantForceShoot || hubCounts;
    boolean aimed =
        turret.atGoal()
            && shooter.isReady()
            && solution.inRange()
            && Math.abs(v.omegaRadiansPerSecond) < MAX_FIRE_OMEGA;
    firing =
        DriverStation.isEnabled()
            && (!holdFire || wantForceShoot || wantPass)
            && target != Target.NONE
            && legal
            && worth
            && aimed
            && !trench
            && !wantEject;

    // ---------------- indexer
    if (wantEject) {
      indexer.setGoal(Indexer.Goal.REVERSE);
    } else if (firing) {
      indexer.setGoal(Indexer.Goal.FEED);
      if (!indexer.isBallStaged() && now - lastBallSeenTime > EMPTY_TIMEOUT_SEC) {
        holdingFuel = false; // fed until nothing came up for a while: empty
      }
    } else if (wantIntake) {
      indexer.setGoal(Indexer.Goal.AGITATE);
    } else if (holdingFuel) {
      indexer.setGoal(Indexer.Goal.STAGE);
    } else {
      indexer.setGoal(Indexer.Goal.IDLE);
    }

    // ---------------- intake
    if (wantEject) {
      intake.setGoal(Intake.Goal.EJECT);
    } else if (wantIntake) {
      intake.setGoal(Intake.Goal.INTAKE);
    } else if (intake.getGoal() != Intake.Goal.HOME) {
      intake.setGoal(Intake.Goal.STOW);
    }

    Logger.recordOutput("Superstructure/Target", target);
    Logger.recordOutput("Superstructure/AimPoint", new Pose2d(aimPoint, Rotation2d.kZero));
    Logger.recordOutput(
        "Superstructure/VirtualTarget",
        new Pose2d(solution.aimX(), solution.aimY(), Rotation2d.kZero));
    Logger.recordOutput("Superstructure/DistanceM", solution.distance());
    Logger.recordOutput("Superstructure/TofSec", solution.tofSec());
    Logger.recordOutput("Superstructure/InRange", solution.inRange());
    Logger.recordOutput("Superstructure/InZone", inZone);
    Logger.recordOutput("Superstructure/HubCounts", hubCounts);
    Logger.recordOutput("Superstructure/HubSoon", hubSoon);
    Logger.recordOutput("Superstructure/Trench", trench);
    Logger.recordOutput("Superstructure/Fire", firing);
    Logger.recordOutput("Superstructure/AutoWinner", winner);
  }

  /**
   * How far (m, along X) the bumper corner nearest our ALLIANCE WALL is outside our ALLIANCE
   * ZONE; negative when some part of the BUMPERS is inside (G407 allows launching into the HUB).
   */
  static double zoneDepthOutside(Pose2d pose) {
    double hl = RobotConstants.FRAME_LENGTH_M / 2 + RobotConstants.BUMPER_DEPTH_M;
    double hw = RobotConstants.FRAME_WIDTH_M / 2 + RobotConstants.BUMPER_DEPTH_M;
    double c = Math.abs(pose.getRotation().getCos());
    double s = Math.abs(pose.getRotation().getSin());
    double halfExtentX = hl * c + hw * s;
    double nearestX = pose.getX() + (FieldConstants.isRed() ? halfExtentX : -halfExtentX);
    return FieldConstants.distanceOutsideAllianceZone(new Translation2d(nearestX, pose.getY()));
  }

  /** True when the robot is in, or within {@link #TRENCH_LOOKAHEAD_SEC} of, a TRENCH. */
  public static boolean nearTrench(Pose2d pose, ChassisSpeeds fieldVel) {
    double px = pose.getX() + fieldVel.vxMetersPerSecond * TRENCH_LOOKAHEAD_SEC;
    double py = pose.getY() + fieldVel.vyMetersPerSecond * TRENCH_LOOKAHEAD_SEC;
    double halfX = FieldConstants.TRENCH_DEPTH / 2 + TRENCH_HALF_LENGTH_M;
    for (double tx :
        new double[] {FieldConstants.TRENCH_X, FieldConstants.FIELD_LENGTH - FieldConstants.TRENCH_X}) {
      boolean xIn = Math.abs(px - tx) < halfX || Math.abs(pose.getX() - tx) < halfX;
      double minY = Math.min(py, pose.getY());
      double maxY = Math.max(py, pose.getY());
      boolean yIn = minY < TRENCH_LANE_EDGE_M || maxY > FieldConstants.FIELD_WIDTH - TRENCH_LANE_EDGE_M;
      if (xIn && yIn) {
        return true;
      }
    }
    return false;
  }

  private static Translation2d nearestPassTarget(Translation2d robot) {
    Translation2d a = FieldConstants.allianceFlip(FieldConstants.BLUE_PASS_LEFT);
    Translation2d b = FieldConstants.allianceFlip(FieldConstants.BLUE_PASS_RIGHT);
    return robot.getDistance(a) < robot.getDistance(b) ? a : b;
  }

  // ---------------- requests (bound to buttons / used by autos)
  public void setWantIntake(boolean v) {
    wantIntake = v;
  }

  public void setWantForceShoot(boolean v) {
    wantForceShoot = v;
  }

  public void setWantPass(boolean v) {
    wantPass = v;
  }

  public void setWantEject(boolean v) {
    wantEject = v;
  }

  public void setHoldFire(boolean v) {
    holdFire = v;
  }

  public void toggleAutoPass() {
    autoPass = !autoPass;
  }

  /** Operator trim for the whole shot table (e.g. worn FUEL or a different carpet), +/-5 % max. */
  public void adjustSpeedTrim(double delta) {
    speedTrim = Math.max(-0.05, Math.min(0.05, speedTrim + delta));
  }

  @AutoLogOutput(key = "Superstructure/SpeedTrim")
  public double getSpeedTrim() {
    return speedTrim;
  }

  /** Mark the robot as loaded (start of a match, or after a manual reload). */
  public void setHoldingFuel(boolean v) {
    holdingFuel = v;
    lastBallSeenTime = Timer.getFPGATimestamp();
  }

  public Command intakeWhileHeld() {
    return Commands.startEnd(() -> setWantIntake(true), () -> setWantIntake(false)).withName("Intake");
  }

  public Command forceShootWhileHeld() {
    return Commands.startEnd(() -> setWantForceShoot(true), () -> setWantForceShoot(false))
        .withName("ForceShoot");
  }

  public Command passWhileHeld() {
    return Commands.startEnd(() -> setWantPass(true), () -> setWantPass(false)).withName("Pass");
  }

  public Command ejectWhileHeld() {
    return Commands.startEnd(() -> setWantEject(true), () -> setWantEject(false)).withName("Eject");
  }

  public Command holdFireWhileHeld() {
    return Commands.startEnd(() -> setHoldFire(true), () -> setHoldFire(false)).withName("HoldFire");
  }

  @AutoLogOutput(key = "Superstructure/HoldingFuel")
  public boolean isHoldingFuel() {
    return holdingFuel;
  }

  @AutoLogOutput(key = "Superstructure/AutoPass")
  public boolean isAutoPass() {
    return autoPass;
  }

  public boolean isFiring() {
    return firing;
  }

  public Target getTarget() {
    return target;
  }

  public ShotSolver.Solution getSolution() {
    return solution;
  }
}
