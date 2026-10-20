package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Shoot-on-the-move: the ball, carrying the robot's velocity, must still arrive at the HUB. */
class ShotSolverTest {
  private static final double RELEASE_Z = 18.2 * 0.0254;
  private static final double OPENING_Z = 72.0 * 0.0254;
  private static final double OFF_X = -4.5 * 0.0254;
  private final ShotSolver solver = new ShotSolver(ShotTables.hub(), OFF_X, 0.0, 0.0);

  private double[] fly(ShotSolver.Input in, ShotSolver.Solution s) {
    double c = Math.cos(in.heading());
    double sn = Math.sin(in.heading());
    double ox = c * OFF_X;
    double oy = sn * OFF_X;
    double vx = in.vx() - in.omega() * oy;
    double vy = in.vy() + in.omega() * ox;
    double[] b =
        BallFlight.launch(
            in.x() + ox, in.y() + oy, RELEASE_Z, s.exitSpeedMps(), Math.toRadians(s.launchDeg()),
            s.turretFieldHeading(), vx, vy);
    return BallFlight.crossDescending(b, OPENING_Z, 0.002);
  }

  @Test
  void stationaryShotIsAimedAtTheTarget() {
    var in = new ShotSolver.Input(1.0, 1.0, 0.3, 0, 0, 0, 4.0, 3.0);
    var s = solver.solve(in);
    assertTrue(s.converged());
    double[] hit = fly(in, s);
    assertNotNull(hit);
    assertEquals(4.0, hit[0], 0.08);
    assertEquals(3.0, hit[1], 0.08);
  }

  @Test
  void movingShotsLandInTheOpening() {
    double safeHalf = 41.7 * 0.0254 / 2 - BallFlight.FUEL_D / 2;
    double[][] cases = {
      {1.5, 2.0, 0.0, 2.0, 0.0, 0.0},
      {1.5, 6.0, 1.0, 0.0, -2.5, 0.0},
      {2.5, 1.5, -0.5, -1.5, 1.5, 1.0},
      {0.8, 4.0, 3.1, 1.0, 1.0, -2.0},
    };
    for (double[] k : cases) {
      var in = new ShotSolver.Input(k[0], k[1], k[2], k[3], k[4], k[5], 4.6255, 4.0345);
      var s = solver.solve(in);
      assertTrue(s.converged(), "converged");
      double[] hit = fly(in, s);
      assertNotNull(hit);
      double miss = Math.hypot(hit[0] - 4.6255, hit[1] - 4.0345);
      assertTrue(miss < safeHalf * 0.6, "miss " + miss + " m for case " + java.util.Arrays.toString(k));
    }
  }

  @Test
  void wrapStaysInRange() {
    for (double a = -20; a < 20; a += 0.37) {
      double w = ShotSolver.wrap(a);
      assertTrue(w > -Math.PI - 1e-12 && w <= Math.PI + 1e-12);
      assertEquals(Math.sin(a), Math.sin(w), 1e-9);
    }
  }
}
