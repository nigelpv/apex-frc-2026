package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CanIds;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX rotor = new TalonFX(CanIds.SERIALIZER);
  private final TalonFX uptake = new TalonFX(CanIds.UPTAKE);
  private final CANrange uptakeSensor = new CANrange(CanIds.CANRANGE_UPTAKE);
  private final CANrange hopperSensor = new CANrange(CanIds.CANRANGE_HOPPER);

  private final VelocityVoltage rotorVelocity = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage uptakeVelocity = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final VoltageOut voltage = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> rotorPos = rotor.getPosition();
  private final StatusSignal<AngularVelocity> rotorVel = rotor.getVelocity();
  private final StatusSignal<Voltage> rotorVolts = rotor.getMotorVoltage();
  private final StatusSignal<Current> rotorAmps = rotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> uptakeVel = uptake.getVelocity();
  private final StatusSignal<Voltage> uptakeVolts = uptake.getMotorVoltage();
  private final StatusSignal<Current> uptakeAmps = uptake.getStatorCurrent();
  private final StatusSignal<Distance> uptakeDist = uptakeSensor.getDistance();
  private final StatusSignal<Boolean> uptakeDetect = uptakeSensor.getIsDetected();
  private final StatusSignal<Distance> hopperDist = hopperSensor.getDistance();
  private final StatusSignal<Boolean> hopperDetect = hopperSensor.getIsDetected();

  public IndexerIOTalonFX() {
    var rotorConfig = new TalonFXConfiguration();
    rotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rotorConfig.Feedback.SensorToMechanismRatio = ROTOR_RATIO;
    rotorConfig.CurrentLimits.StatorCurrentLimit = 80.0; // 60 FUEL jam breaker
    rotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotorConfig.Slot0.kP = 2.0; // V per rotor rps error
    rotorConfig.Slot0.kV = 1.24; // 12 V / (5800 rpm / 60 / 13.33)
    rotorConfig.Slot0.kS = 0.25;
    tryUntilOk(5, () -> rotor.getConfigurator().apply(rotorConfig, 0.25));

    var uptakeConfig = new TalonFXConfiguration();
    uptakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    uptakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    uptakeConfig.Feedback.SensorToMechanismRatio = UPTAKE_RATIO;
    uptakeConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    uptakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    uptakeConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    uptakeConfig.Slot0.kP = 0.3;
    uptakeConfig.Slot0.kV = 0.0956; // 12 V / (7530 rpm / 60)
    uptakeConfig.Slot0.kS = 0.2;
    tryUntilOk(5, () -> uptake.getConfigurator().apply(uptakeConfig, 0.25));

    var rangeConfig = new CANrangeConfiguration();
    rangeConfig.ProximityParams.ProximityThreshold = UPTAKE_BALL_DIST_M;
    rangeConfig.ProximityParams.ProximityHysteresis = 0.01;
    tryUntilOk(5, () -> uptakeSensor.getConfigurator().apply(rangeConfig, 0.25));
    var hopperRangeConfig = new CANrangeConfiguration();
    hopperRangeConfig.ProximityParams.ProximityThreshold = HOPPER_FULL_DIST_M;
    hopperRangeConfig.ProximityParams.ProximityHysteresis = 0.02;
    tryUntilOk(5, () -> hopperSensor.getConfigurator().apply(hopperRangeConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rotorPos, rotorVel, rotorVolts, rotorAmps, uptakeVel, uptakeVolts, uptakeAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, uptakeDist, uptakeDetect, hopperDist, hopperDetect);
    ParentDevice.optimizeBusUtilizationForAll(rotor, uptake, uptakeSensor, hopperSensor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.rotorConnected = BaseStatusSignal.refreshAll(rotorPos, rotorVel, rotorVolts, rotorAmps).isOK();
    inputs.rotorPositionRot = rotorPos.getValueAsDouble();
    inputs.rotorVelocityRps = rotorVel.getValueAsDouble();
    inputs.rotorAppliedVolts = rotorVolts.getValueAsDouble();
    inputs.rotorCurrentAmps = rotorAmps.getValueAsDouble();

    inputs.uptakeConnected = BaseStatusSignal.refreshAll(uptakeVel, uptakeVolts, uptakeAmps).isOK();
    inputs.uptakeVelocityRps = uptakeVel.getValueAsDouble();
    inputs.uptakeAppliedVolts = uptakeVolts.getValueAsDouble();
    inputs.uptakeCurrentAmps = uptakeAmps.getValueAsDouble();

    inputs.uptakeSensorConnected = BaseStatusSignal.refreshAll(uptakeDist, uptakeDetect).isOK();
    inputs.uptakeSensorDistanceM = uptakeDist.getValueAsDouble();
    inputs.uptakeBallDetected = uptakeDetect.getValue();
    inputs.hopperSensorConnected = BaseStatusSignal.refreshAll(hopperDist, hopperDetect).isOK();
    inputs.hopperSensorDistanceM = hopperDist.getValueAsDouble();
    inputs.hopperFullDetected = hopperDetect.getValue();
  }

  @Override
  public void setRotorVelocity(double rps) {
    rotor.setControl(rotorVelocity.withVelocity(rps));
  }

  @Override
  public void setRotorVoltage(double volts) {
    rotor.setControl(voltage.withOutput(volts));
  }

  @Override
  public void setUptakeVelocity(double rps) {
    uptake.setControl(uptakeVelocity.withVelocity(rps));
  }

  @Override
  public void setUptakeVoltage(double volts) {
    uptake.setControl(voltage.withOutput(volts));
  }
}
