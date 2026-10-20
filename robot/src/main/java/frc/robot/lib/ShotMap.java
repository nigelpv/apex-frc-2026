package frc.robot.lib;

/**
 * Distance-indexed ballistic lookup table (pure Java, no WPILib).
 *
 * <p>Rows come from analysis/ballistics.py (drag + Magnus, Monte-Carlo optimised launch angle).
 * Every row is: horizontal distance from the launch point to the aim point (m), hood launch angle
 * (deg above horizontal), ball exit speed (m/s) and time of flight (s). Values between rows are
 * linearly interpolated; queries outside the table are clamped and flagged as out of range.
 */
public final class ShotMap {
  /** One interpolated shot. */
  public record Shot(double launchDeg, double exitSpeedMps, double tofSec, boolean inRange) {}

  private final double[] dist;
  private final double[] launchDeg;
  private final double[] exitMps;
  private final double[] tof;

  public ShotMap(double[][] rows) {
    if (rows.length < 2) {
      throw new IllegalArgumentException("ShotMap needs at least two rows");
    }
    dist = new double[rows.length];
    launchDeg = new double[rows.length];
    exitMps = new double[rows.length];
    tof = new double[rows.length];
    for (int i = 0; i < rows.length; i++) {
      if (rows[i].length != 4) {
        throw new IllegalArgumentException("row " + i + " must be {dist, launchDeg, exit, tof}");
      }
      if (i > 0 && rows[i][0] <= rows[i - 1][0]) {
        throw new IllegalArgumentException("distances must be strictly increasing");
      }
      dist[i] = rows[i][0];
      launchDeg[i] = rows[i][1];
      exitMps[i] = rows[i][2];
      tof[i] = rows[i][3];
    }
  }

  public double minDistance() {
    return dist[0];
  }

  public double maxDistance() {
    return dist[dist.length - 1];
  }

  /** Interpolated shot at the given horizontal distance. */
  public Shot get(double d) {
    boolean inRange = d >= dist[0] && d <= dist[dist.length - 1];
    double q = Math.max(dist[0], Math.min(dist[dist.length - 1], d));
    int hi = 1;
    while (hi < dist.length - 1 && dist[hi] < q) {
      hi++;
    }
    int lo = hi - 1;
    double t = (q - dist[lo]) / (dist[hi] - dist[lo]);
    return new Shot(
        lerp(launchDeg[lo], launchDeg[hi], t),
        lerp(exitMps[lo], exitMps[hi], t),
        lerp(tof[lo], tof[hi], t),
        inRange);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
