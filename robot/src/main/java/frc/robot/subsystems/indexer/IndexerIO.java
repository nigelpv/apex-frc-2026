package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Rotor serializer (X60, 13.33:1) + twin-belt uptake (X44 direct) + two CANrange sensors. */
public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean rotorConnected = false;
    public double rotorPositionRot = 0.0;
    public double rotorVelocityRps = 0.0;
    public double rotorAppliedVolts = 0.0;
    public double rotorCurrentAmps = 0.0;

    public boolean uptakeConnected = false;
    public double uptakeVelocityRps = 0.0;
    public double uptakeAppliedVolts = 0.0;
    public double uptakeCurrentAmps = 0.0;

    public boolean uptakeSensorConnected = false;
    public double uptakeSensorDistanceM = 1.0;
    public boolean uptakeBallDetected = false;

    public boolean hopperSensorConnected = false;
    public double hopperSensorDistanceM = 1.0;
    public boolean hopperFullDetected = false;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  /** Rotor speed in rotor rotations per second (closed loop). */
  default void setRotorVelocity(double rps) {}

  default void setRotorVoltage(double volts) {}

  /** Uptake belt shaft speed in rotations per second (closed loop). */
  default void setUptakeVelocity(double rps) {}

  default void setUptakeVoltage(double volts) {}
}
