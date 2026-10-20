package frc.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Motor models used by the physics sims. */
public final class Motors {
  /** WCP Kraken X44 (Talon FX integrated), trapezoidal commutation. */
  public static DCMotor krakenX44(int count) {
    return new DCMotor(
        12.0, 4.11, 279.0, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530.0), count);
  }

  /** WCP Kraken X60 (Talon FX integrated), FOC. */
  public static DCMotor krakenX60Foc(int count) {
    return DCMotor.getKrakenX60Foc(count);
  }

  private Motors() {}
}
