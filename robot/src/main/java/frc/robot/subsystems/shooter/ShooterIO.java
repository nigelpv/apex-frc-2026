package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Flywheel (2x Kraken X60 1:1, one per side of the shaft) and hood (X44, 70.3:1 sector). */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double flywheelVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftTempC = 0.0;
    public double rightTempC = 0.0;

    public boolean hoodConnected = false;
    /** Hood lift above the down stop (0 = 45 deg launch, TRENCH safe). */
    public double hoodPositionRad = 0.0;
    public double hoodVelocityRadPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Closed-loop flywheel speed (torque-current FOC on the real robot). */
  default void setFlywheelVelocity(double radPerSec) {}

  default void setFlywheelVoltage(double volts) {}

  default void setHoodPosition(double liftRad) {}

  default void setHoodVoltage(double volts) {}

  default void resetHoodPosition(double liftRad) {}
}
