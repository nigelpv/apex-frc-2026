package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** The shot tables must agree with the flight model, and stay smooth enough to interpolate. */
class ShotTablesTest {
  private static final double RELEASE_Z = 18.2 * 0.0254;
  private static final double OPENING_Z = 72.0 * 0.0254;

  @Test
  void everyHubRowDropsThroughTheOpening() {
    for (double[] row : ShotTables.HUB) {
      double[] b = BallFlight.launch(0, 0, RELEASE_Z, row[2], Math.toRadians(row[1]), 0, 0, 0);
      double[] hit = BallFlight.crossDescending(b, OPENING_Z, 0.002);
      assertNotNull(hit, "row " + row[0]);
      assertEquals(row[0], hit[0], 0.06, "landing distance for row " + row[0]);
      assertEquals(row[3], hit[2], 0.03, "time of flight for row " + row[0]);
    }
  }

  @Test
  void interpolatedShotsStillScore() {
    ShotMap map = ShotTables.hub();
    double safeHalf = 41.7 * 0.0254 / 2 - BallFlight.FUEL_D / 2 - 0.03;
    for (double d = map.minDistance(); d <= map.maxDistance(); d += 0.05) {
      ShotMap.Shot s = map.get(d);
      double[] b = BallFlight.launch(0, 0, RELEASE_Z, s.exitSpeedMps(), Math.toRadians(s.launchDeg()), 0, 0, 0);
      double[] hit = BallFlight.crossDescending(b, OPENING_Z, 0.002);
      assertNotNull(hit);
      assertEquals(d, hit[0], safeHalf * 0.5, "interpolated shot at " + d);
    }
  }

  @Test
  void hoodAngleNeverRisesWithDistance() {
    for (int i = 1; i < ShotTables.HUB.length; i++) {
      assertTrue(ShotTables.HUB[i][1] <= ShotTables.HUB[i - 1][1]);
    }
  }

  @Test
  void everyShotIsWithinTheShooter() {
    double maxExit = 5600.0 / 60.0 * Math.PI * 4 * 0.0254 * 0.47;
    for (double[] row : ShotTables.HUB) {
      assertTrue(row[1] >= 45 && row[1] <= 70);
      assertTrue(row[2] < maxExit);
    }
    for (double[] row : ShotTables.PASS) {
      assertTrue(row[2] < maxExit);
    }
  }
}
