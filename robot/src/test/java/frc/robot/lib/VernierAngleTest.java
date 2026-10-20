package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;
import org.junit.jupiter.api.Test;

/** Turret absolute angle from the 19T / 21T CANcoder pair on the 120T ring. */
class VernierAngleTest {
  private final VernierAngle vernier = new VernierAngle(120, 19, 21, -0.56, 0.56);

  @Test
  void uniqueRangeCoversTheCableChain() {
    assertTrue(vernier.uniqueRangeTurns() * 360.0 > 2 * 200.0);
  }

  @Test
  void solvesWithEncoderNoise() {
    Random r = new Random(1);
    double worst = 0;
    for (int i = 0; i < 20000; i++) {
      double t = (-190 + 380 * r.nextDouble()) / 360.0;
      double ea = t * 120 / 19 + r.nextGaussian() * 0.004;
      double eb = t * 120 / 21 + r.nextGaussian() * 0.004;
      VernierAngle.Result res = vernier.solve(ea, eb);
      assertTrue(res.valid());
      worst = Math.max(worst, Math.abs(res.turretTurns() - t) * 360);
    }
    assertTrue(worst < 1.0, "worst error " + worst + " deg");
  }

  @Test
  void rejectsReadingsBetweenCandidates() {
    // encoder B half a candidate spacing (0.0476 rev) away from any consistent reading
    for (double t = -0.5; t <= 0.5; t += 0.05) {
      VernierAngle.Result res = vernier.solve(t * 120 / 19, t * 120 / 21 + 0.0476);
      assertFalse(res.valid(), "accepted a bad reading at " + t);
    }
  }
}
