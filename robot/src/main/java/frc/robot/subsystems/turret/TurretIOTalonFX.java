package frc.robot.subsystems.turret;

import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CanIds;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor = new TalonFX(CanIds.TURRET);
  private final CANcoder encA = new CANcoder(CanIds.TURRET_ENC_19T);
  private final CANcoder encB = new CANcoder(CanIds.TURRET_ENC_21T);

  private final PositionVoltage track = new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final MotionMagicVoltage slewRequest = new MotionMagicVoltage(0.0).withSlot(1).withEnableFOC(true);
  private final VoltageOut voltage = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> applied = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();
  private final StatusSignal<Angle> absA = encA.getAbsolutePosition();
  private final StatusSignal<Angle> absB = encB.getAbsolutePosition();

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public TurretIOTalonFX() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = MOTOR_TO_TURRET;
    config.CurrentLimits.StatorCurrentLimit = 50.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 35.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Slot 0: fine tracking (PositionVoltage + velocity feedforward), units V per turret rotation
    config.Slot0.kP = 90.0;
    config.Slot0.kD = 2.0;
    config.Slot0.kV = 6.7; // 12 V at 7530 rpm / 70 = 1.79 turret rps
    config.Slot0.kS = 0.22;
    // Slot 1: large moves under Motion Magic
    config.Slot1.kP = 60.0;
    config.Slot1.kD = 1.0;
    config.Slot1.kV = 6.7;
    config.Slot1.kS = 0.22;
    config.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(MAX_VEL_RAD_PER_SEC);
    config.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(MAX_ACCEL_RAD_PER_SEC2);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(MAX_RAD);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(MIN_RAD);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, applied, current);
    BaseStatusSignal.setUpdateFrequencyForAll(20.0, absA, absB);
    ParentDevice.optimizeBusUtilizationForAll(motor, encA, encB);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(position, velocity, applied, current).isOK();
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = applied.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.encAConnected = BaseStatusSignal.refreshAll(absA).isOK();
    inputs.encBConnected = BaseStatusSignal.refreshAll(absB).isOK();
    inputs.encARot = absA.getValueAsDouble();
    inputs.encBRot = absB.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRad, double velocityRadPerSec, boolean slew) {
    if (slew) {
      motor.setControl(slewRequest.withPosition(Units.radiansToRotations(positionRad)));
    } else {
      motor.setControl(
          track
              .withPosition(Units.radiansToRotations(positionRad))
              .withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltage.withOutput(volts));
  }

  @Override
  public void resetPosition(double positionRad) {
    motor.setPosition(Units.radiansToRotations(positionRad));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config.MotorOutput, 0.0);
  }
}
