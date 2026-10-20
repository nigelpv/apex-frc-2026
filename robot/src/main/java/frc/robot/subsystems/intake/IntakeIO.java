package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the over-bumper intake (deploy arm + roller train). */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean deployConnected = false;
    /** Arm angle: 0 = deployed (37 deg below horizontal), 127 deg = stowed (vertical). */
    public double deployPositionRad = 0.0;
    public double deployVelocityRadPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;

    public boolean rollerConnected = false;
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Closed-loop arm position (Motion Magic) with an arbitrary feedforward voltage. */
  default void setDeployPosition(double positionRad, double feedforwardVolts) {}

  default void setDeployVoltage(double volts) {}

  /** Stator limit for the deploy motor (low while deployed so the arm yields on impact). */
  default void setDeployStatorLimit(double amps) {}

  /** Tell the deploy encoder where the arm is (used after homing against the stow stop). */
  default void resetDeployPosition(double positionRad) {}

  default void setRollerVoltage(double volts) {}
}
