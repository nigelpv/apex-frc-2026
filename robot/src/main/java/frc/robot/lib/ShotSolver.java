package frc.robot.lib;

/**
 * Shoot-on-the-move solver (pure Java, no WPILib).
 *
 * <p>The ballistic table is solved for a stationary launcher. A moving launcher adds its own
 * velocity to the ball, so we aim at a "virtual target" displaced by -v * tof and iterate because
 * the time of flight depends on the distance to that virtual target. Typical convergence is 3 to 4
 * iterations to under a millisecond.
 *
 * <p>Conventions: field frame (WPILib blue origin), metres, radians, counter-clockwise positive.
 * The turret axis sits at (offX, offY) in the robot frame.
 */
public final class ShotSolver {

  /** Robot state used by the solver. Velocities are field-relative. */
  public record Input(
      double x,
      double y,
      double heading,
      double vx,
      double vy,
      double omega,
      double targetX,
      double targetY) {}

  /** Result of one solve. Angles in radians unless the name says degrees. */
  public record Solution(
      double turretFieldHeading,
      double turretRobotAngle,
      double turretRobotRate,
      double launchDeg,
      double exitSpeedMps,
      double tofSec,
      double distance,
      double launcherX,
      double launcherY,
      double aimX,
      double aimY,
      boolean inRange,
      boolean converged,
      int iterations) {}

  private final ShotMap map;
  private final double offX;
  private final double offY;
  private final double latencySec;

  /**
   * @param map ballistic table for this target type
   * @param offX turret axis X in the robot frame (m)
   * @param offY turret axis Y in the robot frame (m)
   * @param latencySec time from "solve" to the ball leaving the flywheel (control + feed delay)
   */
  public ShotSolver(ShotMap map, double offX, double offY, double latencySec) {
    this.map = map;
    this.offX = offX;
    this.offY = offY;
    this.latencySec = latencySec;
  }

  public Solution solve(Input in) {
    // 1. predict the robot forward by the release latency
    double heading = in.heading() + in.omega() * latencySec;
    double rx = in.x() + in.vx() * latencySec;
    double ry = in.y() + in.vy() * latencySec;

    // 2. launcher (turret axis) position and velocity in the field frame
    double c = Math.cos(heading);
    double s = Math.sin(heading);
    double ox = c * offX - s * offY;
    double oy = s * offX + c * offY;
    double px = rx + ox;
    double py = ry + oy;
    double vpx = in.vx() - in.omega() * oy;
    double vpy = in.vy() + in.omega() * ox;

    // 3. virtual-target iteration
    double tof = map.get(Math.hypot(in.targetX() - px, in.targetY() - py)).tofSec();
    ShotMap.Shot shot = null;
    double gx = in.targetX();
    double gy = in.targetY();
    double d = 0.0;
    boolean converged = false;
    int it = 0;
    for (; it < 8; it++) {
      gx = in.targetX() - vpx * tof;
      gy = in.targetY() - vpy * tof;
      d = Math.hypot(gx - px, gy - py);
      shot = map.get(d);
      double err = Math.abs(shot.tofSec() - tof);
      tof = shot.tofSec();
      if (err < 1e-3) {
        converged = true;
        it++;
        break;
      }
    }

    // 4. turret heading and the feed-forward rate that keeps it on the virtual target
    double rxv = gx - px;
    double ryv = gy - py;
    double fieldHeading = Math.atan2(ryv, rxv);
    double r2 = Math.max(rxv * rxv + ryv * ryv, 1e-6);
    double fieldRate = (-rxv * vpy + ryv * vpx) / r2;
    double robotAngle = wrap(fieldHeading - heading);
    double robotRate = fieldRate - in.omega();

    return new Solution(
        fieldHeading,
        robotAngle,
        robotRate,
        shot.launchDeg(),
        shot.exitSpeedMps(),
        tof,
        d,
        px,
        py,
        gx,
        gy,
        shot.inRange(),
        converged,
        it);
  }

  /** Wrap an angle to (-pi, pi]. */
  public static double wrap(double a) {
    double r = Math.IEEEremainder(a, 2.0 * Math.PI);
    return r <= -Math.PI ? r + 2.0 * Math.PI : r;
  }

  /**
   * Flywheel speed for a requested exit speed.
   *
   * @param exitMps ball exit speed
   * @param wheelDiaM flywheel diameter
   * @param efficiency exit speed divided by wheel surface speed (measured, about 0.88)
   */
  public static double flywheelRpm(double exitMps, double wheelDiaM, double efficiency) {
    return exitMps / (efficiency * Math.PI * wheelDiaM) * 60.0;
  }
}
