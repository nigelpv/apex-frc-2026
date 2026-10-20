package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CanIds;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left = new TalonFX(CanIds.FLYWHEEL_LEFT);
  private final TalonFX right = new TalonFX(CanIds.FLYWHEEL_RIGHT);
  private final TalonFX hood = new TalonFX(CanIds.HOOD);

  private final VelocityTorqueCurrentFOC flywheelRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final VoltageOut voltage = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0.0).withSlot(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> leftVel = left.getVelocity();
  private final StatusSignal<Voltage> leftVolts = left.getMotorVoltage();
  private final StatusSignal<Voltage> rightVolts = right.getMotorVoltage();
  private final StatusSignal<Current> leftAmps = left.getStatorCurrent();
  private final StatusSignal<Current> rightAmps = right.getStatorCurrent();
  private final StatusSignal<Temperature> leftTemp = left.getDeviceTemp();
  private final StatusSignal<Temperature> rightTemp = right.getDeviceTemp();
  private final StatusSignal<Angle> hoodPos = hood.getPosition();
  private final StatusSignal<AngularVelocity> hoodVel = hood.getVelocity();
  private final StatusSignal<Voltage> hoodVolts = hood.getMotorVoltage();
  private final StatusSignal<Current> hoodAmps = hood.getStatorCurrent();

  public ShooterIOTalonFX() {
    var fw = new TalonFXConfiguration();
    fw.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    fw.Feedback.SensorToMechanismRatio = FLYWHEEL_RATIO;
    fw.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    fw.TorqueCurrent.PeakReverseTorqueCurrent = 0.0; // never brake the wheel with current
    fw.CurrentLimits.StatorCurrentLimit = 80.0;
    fw.CurrentLimits.StatorCurrentLimitEnable = true;
    fw.CurrentLimits.SupplyCurrentLimit = 50.0;
    fw.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Torque-current gains: amps per rps error, kS in amps, kV ~0 (torque control)
    fw.Slot0.kP = 6.0;
    fw.Slot0.kS = 3.0;
    fw.Slot0.kV = 0.0;
    fw.Slot0.kA = 0.0;
    // The motors sit on opposite ends of the flywheel shaft facing each other
    fw.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> left.getConfigurator().apply(fw, 0.25));
    fw.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> right.getConfigurator().apply(fw, 0.25));

    var hc = new TalonFXConfiguration();
    hc.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hc.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hc.Feedback.SensorToMechanismRatio = HOOD_RATIO;
    hc.CurrentLimits.StatorCurrentLimit = 25.0;
    hc.CurrentLimits.StatorCurrentLimitEnable = true;
    hc.Slot0.kP = 120.0; // V per hood rotation
    hc.Slot0.kD = 2.0;
    hc.Slot0.kV = 7.5;
    hc.Slot0.kS = 0.15;
    hc.Slot0.kG = 0.1;
    hc.MotionMagic.MotionMagicCruiseVelocity = 0.4; // hood rot/s (25 deg in ~0.2 s)
    hc.MotionMagic.MotionMagicAcceleration = 4.0;
    hc.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hc.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(MAX_LAUNCH_DEG - MIN_LAUNCH_DEG);
    hc.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hc.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    tryUntilOk(5, () -> hood.getConfigurator().apply(hc, 0.25));
    // the hood starts on its down stop (starting configuration)
    tryUntilOk(5, () -> hood.setPosition(0.0, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, leftVel, leftAmps, rightAmps, hoodPos, hoodVel);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftVolts, rightVolts, hoodVolts, hoodAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, leftTemp, rightTemp);
    ParentDevice.optimizeBusUtilizationForAll(left, right, hood);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftConnected = BaseStatusSignal.refreshAll(leftVel, leftVolts, leftAmps, leftTemp).isOK();
    inputs.rightConnected = BaseStatusSignal.refreshAll(rightVolts, rightAmps, rightTemp).isOK();
    inputs.flywheelVelocityRadPerSec = Units.rotationsToRadians(leftVel.getValueAsDouble());
    inputs.leftAppliedVolts = leftVolts.getValueAsDouble();
    inputs.rightAppliedVolts = rightVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftAmps.getValueAsDouble();
    inputs.rightCurrentAmps = rightAmps.getValueAsDouble();
    inputs.leftTempC = leftTemp.getValueAsDouble();
    inputs.rightTempC = rightTemp.getValueAsDouble();

    inputs.hoodConnected = BaseStatusSignal.refreshAll(hoodPos, hoodVel, hoodVolts, hoodAmps).isOK();
    inputs.hoodPositionRad = Units.rotationsToRadians(hoodPos.getValueAsDouble());
    inputs.hoodVelocityRadPerSec = Units.rotationsToRadians(hoodVel.getValueAsDouble());
    inputs.hoodAppliedVolts = hoodVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodAmps.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocity(double radPerSec) {
    double rps = Units.radiansToRotations(radPerSec);
    left.setControl(flywheelRequest.withVelocity(rps));
    right.setControl(flywheelRequest.withVelocity(rps));
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    left.setControl(voltage.withOutput(volts));
    right.setControl(voltage.withOutput(volts));
  }

  @Override
  public void setHoodPosition(double liftRad) {
    hood.setControl(hoodRequest.withPosition(Units.radiansToRotations(liftRad)));
  }

  @Override
  public void setHoodVoltage(double volts) {
    hood.setControl(voltage.withOutput(volts));
  }

  @Override
  public void resetHoodPosition(double liftRad) {
    hood.setPosition(Units.radiansToRotations(liftRad));
  }
}
