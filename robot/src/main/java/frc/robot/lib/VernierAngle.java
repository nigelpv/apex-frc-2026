package frc.robot.lib;

/**
 * Absolute turret angle from two CANcoders geared to the turret ring with coprime-ish tooth
 * counts (Chinese-remainder / vernier). Pure Java so it can be unit tested off-robot.
 *
 * <p>Each encoder reports its shaft angle in rotations [0, 1). With ring teeth R and encoder
 * teeth a and b, a turret angle t (rotations) gives e_a = frac(t R / a) and e_b = frac(t R / b).
 * We search the integer number of encoder-A revolutions k such that t = (k + e_a) a / R predicts
 * e_b best. Neighbouring candidates are 1 / |R/a - R/b| apart in noise margin terms, so the
 * robust range is 1.6625 turret rotations (598.5 deg) for R = 120, a = 19, b = 21: enough for the
 * +/-190 deg turret service loop.
 */
public final class VernierAngle {
  /** Largest encoder-B residual (rotations) accepted as a valid solve. */
  public static final double MAX_RESIDUAL = 0.03;

  private final int ringTeeth;
  private final int teethA;
  private final int teethB;
  private final double minTurns;
  private final double maxTurns;

  /**
   * @param ringTeeth teeth on the turret ring
   * @param teethA encoder A gear teeth
   * @param teethB encoder B gear teeth
   * @param minTurns lowest physically reachable turret angle (rotations, may be negative)
   * @param maxTurns highest physically reachable turret angle (rotations)
   */
  public VernierAngle(int ringTeeth, int teethA, int teethB, double minTurns, double maxTurns) {
    this.ringTeeth = ringTeeth;
    this.teethA = teethA;
    this.teethB = teethB;
    this.minTurns = minTurns;
    this.maxTurns = maxTurns;
  }

  /** Unambiguous turret range in rotations for this gear set. */
  public double uniqueRangeTurns() {
    double diff = Math.abs((double) ringTeeth / teethA - (double) ringTeeth / teethB);
    return 1.0 / diff;
  }

  /** Result of a solve: turret angle (rotations) and the residual on encoder B (rotations). */
  public record Result(double turretTurns, double residual, boolean valid) {}

  /**
   * Solve for the turret angle.
   *
   * @param encA encoder A absolute position, rotations (any value; wrapped to [0, 1))
   * @param encB encoder B absolute position, rotations
   */
  public Result solve(double encA, double encB) {
    double ea = wrap01(encA);
    double eb = wrap01(encB);
    double best = Double.NaN;
    double bestErr = Double.POSITIVE_INFINITY;
    double ratioA = (double) ringTeeth / teethA;
    double ratioB = (double) ringTeeth / teethB;
    int kMin = (int) Math.floor(minTurns * ratioA) - 1;
    int kMax = (int) Math.ceil(maxTurns * ratioA) + 1;
    for (int k = kMin; k <= kMax; k++) {
      double t = (k + ea) / ratioA;
      if (t < minTurns - 0.02 || t > maxTurns + 0.02) {
        continue;
      }
      double predB = wrap01(t * ratioB);
      double err = Math.abs(predB - eb);
      err = Math.min(err, 1.0 - err);
      if (err < bestErr) {
        bestErr = err;
        best = t;
      }
    }
    // Neighbouring candidates sit 0.095 rev apart on encoder B, so any reading is within 0.048 rev
    // of some candidate. Noise plus gear backlash is under 0.006 rev; a residual above 0.03 rev
    // means a slipped gear, a loose magnet or a stale reading. Errors that land near another
    // candidate cannot be caught by one sample, so the turret also requires a stable answer.
    return new Result(best, bestErr, !Double.isNaN(best) && bestErr < MAX_RESIDUAL);
  }

  private static double wrap01(double x) {
    double r = x - Math.floor(x);
    return r >= 1.0 ? 0.0 : r;
  }
}
