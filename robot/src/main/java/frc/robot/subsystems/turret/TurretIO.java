package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/** Turret yaw: X44 + 7:1 MAXPlanetary -> 12T:120T (70:1), two vernier CANcoders (19T, 21T). */
public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    /** Turret angle in the robot frame, 0 = launching toward the intake (+X), CCW positive. */
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public boolean encAConnected = false;
    public boolean encBConnected = false;
    /** Absolute encoder positions, rotations (calibrated so both read 0 at turret 0). */
    public double encARot = 0.0;
    public double encBRot = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Closed-loop position with a velocity feedforward (tracking a moving target).
   *
   * @param positionRad turret angle
   * @param velocityRadPerSec feedforward velocity of the setpoint
   * @param slew true for large moves (Motion Magic profile), false for fine tracking
   */
  default void setPosition(double positionRad, double velocityRadPerSec, boolean slew) {}

  default void setVoltage(double volts) {}

  /** Seed the motor encoder with an absolute turret angle. */
  default void resetPosition(double positionRad) {}

  default void setBrakeMode(boolean brake) {}
}
