package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldConstants;
import frc.robot.lib.BallFlight;
import frc.robot.lib.HubShift;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * FUEL simulation for testing the superstructure in sim: FUEL stored in the robot, pickup from a
 * density model of the field (the NEUTRAL ZONE staging box empties as we take from it), launch
 * with the real shooter / hood / turret state plus robot velocity, flight with the same drag and
 * Magnus model as analysis/ballistics.py, and scoring through the HUB opening with the active-HUB
 * rules. Balls in flight are logged as Translation3d[] for AdvantageScope's 3D field.
 */
public class FuelSim {
  public static final int CAPACITY = 32; // estimate from the CAD hopper volume (design_calcs.py: ~34)
  private static final double FUEL_D = BallFlight.FUEL_D;
  private static final double INTAKE_WIDTH = Units.inchesToMeters(26.5);
  private static final double INTAKE_MAX_BPS = 20.0;
  private static final double FEED_BPS = IndexerConstants.ROTOR_FEED_RPS * IndexerConstants.ROTOR_POCKETS;
  private static final double SCORE_RADIUS = FieldConstants.HUB_OPENING_ACROSS / 2 - FUEL_D / 2;
  private static final double FLYWHEEL_OFFSET = Units.inchesToMeters(4.35);
  private static final double SUBSTEP = 0.005;

  // NEUTRAL ZONE staging box (206 in across the field, 72 in deep) and its FUEL
  private static final double BOX_X0 = FieldConstants.CENTER_X - Units.inchesToMeters(36);
  private static final double BOX_X1 = FieldConstants.CENTER_X + Units.inchesToMeters(36);
  private static final double BOX_Y0 = FieldConstants.FIELD_WIDTH / 2 - Units.inchesToMeters(103);
  private static final double BOX_Y1 = FieldConstants.FIELD_WIDTH / 2 + Units.inchesToMeters(103);
  private static final double BOX_AREA = (BOX_X1 - BOX_X0) * (BOX_Y1 - BOX_Y0);
  private static final double ZONE_AREA = FieldConstants.ALLIANCE_ZONE_DEPTH * FieldConstants.FIELD_WIDTH;

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final IndexerIOSim indexerIO;
  private final ShooterIOSim shooterIO;
  private final Supplier<HubShift.AutoWinner> autoWinner;

  private final List<double[]> inFlight = new ArrayList<>(); // x y z vx vy vz spin(m/s) time
  private int stored = 8;
  private double boxFuel = 360;
  private double ourZoneFuel = 24; // DEPOT
  private double neutralLoose = 40;
  private double pickupAccum = 0.0;
  private double feedAccum = 0.0;
  private double stageTimer = 0.0;
  private int scored = 0;
  private int shots = 0;
  private int missed = 0;

  public FuelSim(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret,
      Shooter shooter,
      IndexerIOSim indexerIO,
      ShooterIOSim shooterIO,
      Supplier<HubShift.AutoWinner> autoWinner) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.turret = turret;
    this.shooter = shooter;
    this.indexerIO = indexerIO;
    this.shooterIO = shooterIO;
    this.autoWinner = autoWinner;
  }

  /** Reset for a new match: preload, refill the field. */
  public void reset() {
    stored = 8;
    boxFuel = 360;
    ourZoneFuel = 24;
    neutralLoose = 40;
    inFlight.clear();
    scored = 0;
    shots = 0;
    missed = 0;
  }

  public void update() {
    double dt = Constants.LOOP_PERIOD_SEC;
    Pose2d pose = drive.getPose();
    ChassisSpeeds field = drive.getFieldVelocity();

    // ---------------- intake
    if (DriverStation.isEnabled()
        && intake.isDeployed()
        && intake.rollerSurfaceSpeed() > 3.0
        && stored < CAPACITY) {
      double c = pose.getRotation().getCos();
      double s = pose.getRotation().getSin();
      double forward = field.vxMetersPerSecond * c + field.vyMetersPerSecond * s;
      double sweep = Math.max(forward, 0.25); // rollers pull in FUEL they touch even when slow
      double density = densityAt(pose.getTranslation());
      pickupAccum += Math.min(INTAKE_MAX_BPS, density * INTAKE_WIDTH * sweep) * dt;
      while (pickupAccum >= 1.0 && stored < CAPACITY) {
        pickupAccum -= 1.0;
        stored++;
        takeFrom(pose.getTranslation());
      }
    } else {
      pickupAccum = 0.0;
    }

    // ---------------- staging sensor + feed
    Indexer.Goal goal = indexer.getGoal();
    boolean moving = goal == Indexer.Goal.STAGE || goal == Indexer.Goal.FEED;
    stageTimer = (moving && stored > 0) ? stageTimer + dt : (stored > 0 ? stageTimer : 0.0);
    boolean staged = stored > 0 && stageTimer > 0.25;
    if (goal == Indexer.Goal.FEED && stored > 0 && DriverStation.isEnabled()) {
      feedAccum += FEED_BPS * dt;
      while (feedAccum >= 1.0 && stored > 0) {
        feedAccum -= 1.0;
        launch(pose, field);
      }
    } else {
      feedAccum = Math.min(feedAccum, 0.0);
    }
    indexerIO.setSensors(staged, stored >= CAPACITY - 4);

    // ---------------- flight
    stepBalls(dt);

    Translation3d[] balls = new Translation3d[inFlight.size()];
    for (int i = 0; i < balls.length; i++) {
      double[] b = inFlight.get(i);
      balls[i] = new Translation3d(b[0], b[1], b[2]);
    }
    Logger.recordOutput("FuelSim/InFlight", balls);
    Logger.recordOutput("FuelSim/Stored", stored);
    Logger.recordOutput("FuelSim/Shots", shots);
    Logger.recordOutput("FuelSim/Scored", scored);
    Logger.recordOutput("FuelSim/Missed", missed);
    Logger.recordOutput("FuelSim/Accuracy", shots == 0 ? 0.0 : (double) scored / shots);
    Logger.recordOutput("FuelSim/NeutralBox", boxFuel);
  }

  private void launch(Pose2d pose, ChassisSpeeds field) {
    stored--;
    shots++;
    double heading = pose.getRotation().getRadians();
    double turretField = heading + turret.getPositionRad();
    double ax = TurretConstants.AXIS_IN_ROBOT.getX();
    double ay = TurretConstants.AXIS_IN_ROBOT.getY();
    double ch = Math.cos(heading);
    double sh = Math.sin(heading);
    double ox = ch * ax - sh * ay;
    double oy = sh * ax + ch * ay;
    double px = pose.getX() + ox + FLYWHEEL_OFFSET * Math.cos(turretField);
    double py = pose.getY() + oy + FLYWHEEL_OFFSET * Math.sin(turretField);
    double w = field.omegaRadiansPerSecond;
    double vrx = field.vxMetersPerSecond - w * oy;
    double vry = field.vyMetersPerSecond + w * ox;

    double v = shooter.getExitSpeedMps();
    double th = Math.toRadians(shooter.getLaunchDeg());
    inFlight.add(
        BallFlight.launch(px, py, ShooterConstants.RELEASE_HEIGHT_M, v, th, turretField, vrx, vry));
    // the ball takes its kinetic energy (plus ~30 % for spin and slip) out of the flywheel
    double rpmNow = Math.max(shooter.getRpm(), 500.0);
    double omega = Units.rotationsPerMinuteToRadiansPerSecond(rpmNow);
    double energy = 1.3 * 0.5 * BallFlight.FUEL_MASS * v * v;
    shooterIO.applyShotImpulse(energy / (ShooterConstants.FLYWHEEL_MOI * omega));
  }

  private void stepBalls(double dt) {
    Iterator<double[]> it = inFlight.iterator();
    while (it.hasNext()) {
      double[] b = it.next();
      boolean done = false;
      for (double t = 0; t < dt - 1e-9 && !done; t += SUBSTEP) {
        double zPrev = b[2];
        BallFlight.step(b, SUBSTEP);

        // descending through the HUB opening plane
        double zo = FieldConstants.HUB_OPENING_Z;
        if (zPrev >= zo && b[2] < zo && b[5] < 0) {
          for (boolean redHub : new boolean[] {false, true}) {
            Translation2d hub =
                redHub ? FieldConstants.flip(FieldConstants.BLUE_HUB) : FieldConstants.BLUE_HUB;
            if (Math.hypot(b[0] - hub.getX(), b[1] - hub.getY()) < SCORE_RADIUS) {
              if (redHub == FieldConstants.isRed() && hubCounts()) {
                scored++;
              } else {
                missed++;
              }
              neutralLoose += 1; // HUB exits return FUEL to the NEUTRAL ZONE
              done = true;
              break;
            }
          }
        }
        if (!done && (b[2] < FUEL_D / 2 || b[7] > 4.0)) {
          missed++;
          Translation2d land = new Translation2d(b[0], b[1]);
          if (FieldConstants.inOurAllianceZone(land)) {
            ourZoneFuel += 1;
          } else {
            neutralLoose += 1;
          }
          done = true;
        }
      }
      if (done) {
        it.remove();
      }
    }
  }

  private boolean hubCounts() {
    if (DriverStation.isAutonomous()) {
      return true;
    }
    double t = DriverStation.getMatchTime();
    return t < 0 || HubShift.ourHubActive(t, autoWinner.get())
        || HubShift.ourHubActive(t + HubShift.SCORING_GRACE, autoWinner.get());
  }

  private boolean inBox(Translation2d p) {
    return p.getX() > BOX_X0 && p.getX() < BOX_X1 && p.getY() > BOX_Y0 && p.getY() < BOX_Y1;
  }

  private double densityAt(Translation2d p) {
    if (inBox(p)) {
      return boxFuel / BOX_AREA + 1.0;
    }
    if (FieldConstants.inOurAllianceZone(p)) {
      return ourZoneFuel / ZONE_AREA;
    }
    double neutralArea = FieldConstants.NEUTRAL_ZONE_DEPTH * FieldConstants.FIELD_WIDTH;
    return neutralLoose / neutralArea;
  }

  private void takeFrom(Translation2d p) {
    if (inBox(p) && boxFuel >= 1) {
      boxFuel -= 1;
    } else if (FieldConstants.inOurAllianceZone(p) && ourZoneFuel >= 1) {
      ourZoneFuel -= 1;
    } else if (neutralLoose >= 1) {
      neutralLoose -= 1;
    }
  }

  public int getStored() {
    return stored;
  }

  public int getScored() {
    return scored;
  }
}
