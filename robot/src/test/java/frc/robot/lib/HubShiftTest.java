package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.lib.HubShift.AutoWinner;
import org.junit.jupiter.api.Test;

/** HUB activation schedule (game manual 6.4): AUTO winner inactive in SHIFT 1 and 3. */
class HubShiftTest {
  @Test
  void transitionAndEndGameAreActiveForBoth() {
    for (AutoWinner w : AutoWinner.values()) {
      assertTrue(HubShift.ourHubActive(135, w));
      assertTrue(HubShift.ourHubActive(20, w));
    }
  }

  @Test
  void autoWinnerSitsOutShiftsOneAndThree() {
    assertFalse(HubShift.ourHubActive(120, AutoWinner.US)); // SHIFT 1
    assertTrue(HubShift.ourHubActive(100, AutoWinner.US)); // SHIFT 2
    assertFalse(HubShift.ourHubActive(70, AutoWinner.US)); // SHIFT 3
    assertTrue(HubShift.ourHubActive(40, AutoWinner.US)); // SHIFT 4
    assertTrue(HubShift.ourHubActive(120, AutoWinner.THEM));
    assertFalse(HubShift.ourHubActive(100, AutoWinner.THEM));
    assertTrue(HubShift.ourHubActive(70, AutoWinner.THEM));
    assertFalse(HubShift.ourHubActive(40, AutoWinner.THEM));
  }

  @Test
  void unknownGameDataNeverWithholdsFire() {
    for (double t = 140; t >= 0; t -= 0.5) {
      assertTrue(HubShift.ourHubActive(t, AutoWinner.UNKNOWN));
    }
  }

  @Test
  void shiftBoundaries() {
    assertEquals(0, HubShift.shiftIndex(130.1));
    assertEquals(1, HubShift.shiftIndex(129.9));
    assertEquals(2, HubShift.shiftIndex(104.9));
    assertEquals(3, HubShift.shiftIndex(55.1));
    assertEquals(4, HubShift.shiftIndex(54.9));
    assertEquals(0, HubShift.shiftIndex(29.9));
  }

  @Test
  void secondsToNextChange() {
    assertEquals(10.0, HubShift.secondsToNextChange(140, AutoWinner.US), 1e-6);
    assertEquals(35.0, HubShift.secondsToNextChange(140, AutoWinner.THEM), 1e-6);
    assertEquals(Double.POSITIVE_INFINITY, HubShift.secondsToNextChange(20, AutoWinner.US));
  }

  @Test
  void graceWindowCountsLateArrivals() {
    // AUTO winner: HUB goes inactive at 130. A ball launched at 131 with 1.2 s flight lands at
    // 129.8, inside the 3 s grace: counts. One landing at 126.5 does not.
    assertTrue(HubShift.shotWillCount(131.0, 1.2, AutoWinner.US));
    assertFalse(HubShift.shotWillCount(128.0, 1.5, AutoWinner.US));
    // a shot fired just before our HUB turns on, landing after it is on, counts
    assertTrue(HubShift.shotWillCount(105.5, 1.0, AutoWinner.US));
  }

  @Test
  void gameDataParsing() {
    assertEquals(AutoWinner.US, HubShift.parseGameData("R", true));
    assertEquals(AutoWinner.THEM, HubShift.parseGameData("R", false));
    assertEquals(AutoWinner.US, HubShift.parseGameData("b", false));
    assertEquals(AutoWinner.UNKNOWN, HubShift.parseGameData("", true));
    assertEquals(AutoWinner.UNKNOWN, HubShift.parseGameData(null, true));
  }
}
