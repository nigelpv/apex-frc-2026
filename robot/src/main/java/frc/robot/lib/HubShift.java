package frc.robot.lib;

/**
 * REBUILT HUB activity schedule (game manual TU22, Table 6-2 / 6-3). Pure Java so it can be unit
 * tested off-robot.
 *
 * <p>TELEOP runs 140 s: TRANSITION SHIFT (140-130 s on the timer, both HUBS active), four 25 s
 * ALLIANCE SHIFTS, END GAME (last 30 s, both active). The ALLIANCE that scored more FUEL in AUTO
 * (game data at the start of TELEOP) has its HUB inactive in SHIFT 1 and SHIFT 3, active in
 * SHIFT 2 and SHIFT 4; the other ALLIANCE is the opposite. FUEL keeps counting for 3 s after a HUB
 * deactivates.
 */
public final class HubShift {
  public static final double TELEOP_LENGTH = 140.0;
  public static final double TRANSITION_END = 130.0;
  public static final double SHIFT_LENGTH = 25.0;
  public static final double END_GAME_START = 30.0;
  public static final double SCORING_GRACE = 3.0;

  /** Which ALLIANCE won AUTO, from the FMS game data. */
  public enum AutoWinner {
    US,
    THEM,
    UNKNOWN
  }

  /** Index of the ALLIANCE SHIFT (1-4) at a TELEOP timer value, or 0 outside the shifts. */
  public static int shiftIndex(double teleopRemaining) {
    if (teleopRemaining > TRANSITION_END || teleopRemaining <= END_GAME_START) {
      return 0;
    }
    int idx = (int) Math.floor((TRANSITION_END - teleopRemaining) / SHIFT_LENGTH) + 1;
    return Math.max(1, Math.min(4, idx));
  }

  /**
   * True when our HUB is active at the given TELEOP timer value.
   *
   * @param teleopRemaining match timer during TELEOP (140 -> 0)
   * @param winner who scored more FUEL in AUTO; UNKNOWN is treated as "always active" so the robot
   *     never withholds fire because of missing game data
   */
  public static boolean ourHubActive(double teleopRemaining, AutoWinner winner) {
    int s = shiftIndex(teleopRemaining);
    if (s == 0 || winner == AutoWinner.UNKNOWN) {
      return true;
    }
    boolean oddShift = (s % 2) == 1;
    // The AUTO winner is inactive in SHIFT 1 and 3.
    return winner == AutoWinner.US ? !oddShift : oddShift;
  }

  /** Seconds until our HUB status next changes (Double.POSITIVE_INFINITY if it never does). */
  public static double secondsToNextChange(double teleopRemaining, AutoWinner winner) {
    boolean now = ourHubActive(teleopRemaining, winner);
    // Step through the boundaries in timer order.
    double[] bounds = {
      TRANSITION_END,
      TRANSITION_END - SHIFT_LENGTH,
      TRANSITION_END - 2 * SHIFT_LENGTH,
      TRANSITION_END - 3 * SHIFT_LENGTH,
      END_GAME_START
    };
    for (double b : bounds) {
      if (b < teleopRemaining - 1e-9) {
        if (ourHubActive(b - 1e-6, winner) != now) {
          return teleopRemaining - b;
        }
      }
    }
    return Double.POSITIVE_INFINITY;
  }

  /**
   * True when a FUEL launched now (arriving after {@code tofSec}) will count: our HUB is active at
   * arrival, or it was active less than {@link #SCORING_GRACE} seconds before arrival.
   */
  public static boolean shotWillCount(double teleopRemaining, double tofSec, AutoWinner winner) {
    double arrival = teleopRemaining - tofSec;
    if (arrival < -SCORING_GRACE) {
      return false;
    }
    if (ourHubActive(arrival, winner)) {
      return true;
    }
    // inactive at arrival: still counts if the HUB went inactive within the grace window
    return ourHubActive(arrival + SCORING_GRACE, winner);
  }

  /** Parse the 2026 FMS game data ('R' or 'B' = ALLIANCE that won AUTO). */
  public static AutoWinner parseGameData(String gameData, boolean weAreRed) {
    if (gameData == null || gameData.isEmpty()) {
      return AutoWinner.UNKNOWN;
    }
    char c = Character.toUpperCase(gameData.charAt(0));
    if (c == 'R') {
      return weAreRed ? AutoWinner.US : AutoWinner.THEM;
    }
    if (c == 'B') {
      return weAreRed ? AutoWinner.THEM : AutoWinner.US;
    }
    return AutoWinner.UNKNOWN;
  }

  private HubShift() {}
}
