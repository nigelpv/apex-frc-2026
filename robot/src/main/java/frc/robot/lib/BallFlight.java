package frc.robot.lib;

/**
 * FUEL flight model (pure Java): point-mass sphere with quadratic drag and backspin Magnus lift,
 * the same physics as analysis/ballistics.py. Used by the FUEL simulator and by the unit tests
 * that check the shot tables against the physics.
 *
 * <p>State array: {x, y, z, vx, vy, vz, spin surface speed (m/s), time in flight (s)}.
 */
public final class BallFlight {
  public static final double FUEL_D = 5.91 * 0.0254;
  public static final double FUEL_MASS = 0.215;
  public static final double RHO = 1.20;
  public static final double CD = 0.47;
  public static final double G = 9.81;
  public static final double SPIN_RATIO = 0.10;
  private static final double K = 0.5 * RHO * Math.PI * FUEL_D * FUEL_D / 4.0 / FUEL_MASS;

  /** New ball state from a launch: speed (m/s), elevation (rad) and horizontal heading (rad). */
  public static double[] launch(
      double x, double y, double z, double speed, double elevation, double heading, double vx0, double vy0) {
    double vh = speed * Math.cos(elevation);
    return new double[] {
      x,
      y,
      z,
      vh * Math.cos(heading) + vx0,
      vh * Math.sin(heading) + vy0,
      speed * Math.sin(elevation),
      SPIN_RATIO * speed,
      0.0
    };
  }

  /** Lift coefficient vs spin ratio (Sawicki-style fit, as in ballistics.py). */
  public static double liftCoefficient(double spinRatio) {
    double s = Math.abs(spinRatio);
    return s > 0 ? s / (2.022 * s + 0.981) : 0.0;
  }

  /** Advance one RK4 step of {@code dt} seconds in place. */
  public static void step(double[] b, double dt) {
    double[] k1 = accel(b[3], b[4], b[5], b[6]);
    double[] k2 = accel(b[3] + k1[0] * dt / 2, b[4] + k1[1] * dt / 2, b[5] + k1[2] * dt / 2, b[6]);
    double[] k3 = accel(b[3] + k2[0] * dt / 2, b[4] + k2[1] * dt / 2, b[5] + k2[2] * dt / 2, b[6]);
    double[] k4 = accel(b[3] + k3[0] * dt, b[4] + k3[1] * dt, b[5] + k3[2] * dt, b[6]);
    b[0] += dt * (b[3] + dt / 6 * (k1[0] + k2[0] + k3[0]));
    b[1] += dt * (b[4] + dt / 6 * (k1[1] + k2[1] + k3[1]));
    b[2] += dt * (b[5] + dt / 6 * (k1[2] + k2[2] + k3[2]));
    for (int i = 0; i < 3; i++) {
      b[3 + i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    b[6] *= 1 - 0.05 * dt;
    b[7] += dt;
  }

  private static double[] accel(double vx, double vy, double vz, double spin) {
    double v = Math.sqrt(vx * vx + vy * vy + vz * vz);
    double vh = Math.hypot(vx, vy);
    double cl = liftCoefficient(spin / Math.max(v, 1e-6));
    // backspin lift is perpendicular to the velocity, in the vertical plane of travel
    double lx = vh > 1e-6 ? -vz * vx / vh : 0.0;
    double ly = vh > 1e-6 ? -vz * vy / vh : 0.0;
    double lz = vh;
    return new double[] {
      -K * CD * v * vx + K * cl * v * lx,
      -K * CD * v * vy + K * cl * v * ly,
      -G - K * CD * v * vz + K * cl * v * lz
    };
  }

  /**
   * Fly a ball until it descends through height {@code zPlane}; returns {x, y, t} at the crossing
   * or null if it never does (within 4 s).
   */
  public static double[] crossDescending(double[] b, double zPlane, double dt) {
    while (b[7] < 4.0) {
      double x0 = b[0];
      double y0 = b[1];
      double z0 = b[2];
      double t0 = b[7];
      step(b, dt);
      if (z0 >= zPlane && b[2] < zPlane && b[5] < 0) {
        double f = (z0 - zPlane) / (z0 - b[2]);
        return new double[] {x0 + f * (b[0] - x0), y0 + f * (b[1] - y0), t0 + f * (b[7] - t0)};
      }
      if (b[2] < 0) {
        return null;
      }
    }
    return null;
  }

  private BallFlight() {}
}
