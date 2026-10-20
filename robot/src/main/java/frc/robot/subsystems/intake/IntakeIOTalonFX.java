package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

/** Intake on two Kraken X44s (roboRIO CAN bus). */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX deploy = new TalonFX(CanIds.INTAKE_DEPLOY);
  private final TalonFX roller = new TalonFX(CanIds.INTAKE_ROLLER);

  private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private final VoltageOut deployVoltage = new VoltageOut(0.0);
  private final VoltageOut rollerVoltage = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> deployPosition = deploy.getPosition();
  private final StatusSignal<AngularVelocity> deployVelocity = deploy.getVelocity();
  private final StatusSignal<Voltage> deployApplied = deploy.getMotorVoltage();
  private final StatusSignal<Current> deployCurrent = deploy.getStatorCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocity = roller.getVelocity();
  private final StatusSignal<Voltage> rollerApplied = roller.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = roller.getStatorCurrent();

  private final TalonFXConfiguration deployConfig = new TalonFXConfiguration();

  public IntakeIOTalonFX() {
    // Deploy arm: position in arm rotations, 0 = deployed
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployConfig.Feedback.SensorToMechanismRatio = DEPLOY_RATIO;
    deployConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    deployConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.Slot0.kP = 45.0; // V per arm rotation
    deployConfig.Slot0.kD = 1.5;
    deployConfig.Slot0.kV = 8.9; // V per arm rot/s (X44 7530 rpm / 75:1)
    deployConfig.Slot0.kS = 0.15;
    deployConfig.MotionMagic.MotionMagicCruiseVelocity = 1.2; // arm rot/s -> 127 deg in ~0.4 s
    deployConfig.MotionMagic.MotionMagicAcceleration = 8.0;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(STOWED_RAD + Units.degreesToRadians(2.0));
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(DEPLOYED_RAD - Units.degreesToRadians(4.0));
    tryUntilOk(5, () -> deploy.getConfigurator().apply(deployConfig, 0.25));
    // The robot starts stowed (frame perimeter rule), so the arm sits on its stow stop at boot.
    tryUntilOk(5, () -> deploy.setPosition(Units.radiansToRotations(STOWED_RAD), 0.25));

    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_RATIO;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        deployPosition,
        deployVelocity,
        deployApplied,
        deployCurrent,
        rollerVelocity,
        rollerApplied,
        rollerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(deploy, roller);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deployConnected =
        BaseStatusSignal.refreshAll(deployPosition, deployVelocity, deployApplied, deployCurrent)
            .isOK();
    inputs.deployPositionRad = Units.rotationsToRadians(deployPosition.getValueAsDouble());
    inputs.deployVelocityRadPerSec = Units.rotationsToRadians(deployVelocity.getValueAsDouble());
    inputs.deployAppliedVolts = deployApplied.getValueAsDouble();
    inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();

    inputs.rollerConnected =
        BaseStatusSignal.refreshAll(rollerVelocity, rollerApplied, rollerCurrent).isOK();
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerApplied.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
  }

  @Override
  public void setDeployPosition(double positionRad, double feedforwardVolts) {
    deploy.setControl(
        deployRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setDeployVoltage(double volts) {
    deploy.setControl(deployVoltage.withOutput(volts));
  }

  @Override
  public void setDeployStatorLimit(double amps) {
    var limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit = amps;
    limits.StatorCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = 30.0;
    limits.SupplyCurrentLimitEnable = true;
    deploy.getConfigurator().apply(limits, 0.0);
  }

  @Override
  public void resetDeployPosition(double positionRad) {
    deploy.setPosition(Units.radiansToRotations(positionRad));
  }

  @Override
  public void setRollerVoltage(double volts) {
    roller.setControl(rollerVoltage.withOutput(volts));
  }
}
