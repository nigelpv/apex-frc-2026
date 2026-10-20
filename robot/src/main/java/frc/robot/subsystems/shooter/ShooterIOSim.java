package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.Motors;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim flywheel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(Motors.krakenX60Foc(2), FLYWHEEL_MOI, FLYWHEEL_RATIO),
          Motors.krakenX60Foc(2));
  private final DCMotorSim hood =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Motors.krakenX44(1), 0.02, HOOD_RATIO),
          Motors.krakenX44(1));
  private final PIDController flywheelPid = new PIDController(0.05, 0.0, 0.0);
  private final PIDController hoodPid = new PIDController(40.0, 0.0, 0.5);

  private Double flywheelSetpoint = null;
  private Double hoodSetpoint = null;
  private double flywheelVolts = 0.0;
  private double hoodVolts = 0.0;

  /** Drag the wheel down when the FUEL sim launches a ball (energy taken by the ball). */
  public void applyShotImpulse(double deltaRadPerSec) {
    flywheel.setAngularVelocity(Math.max(0.0, flywheel.getAngularVelocityRadPerSec() - deltaRadPerSec));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double w = flywheel.getAngularVelocityRadPerSec();
    if (flywheelSetpoint != null) {
      double kV = 12.0 / Units.rotationsPerMinuteToRadiansPerSecond(6000.0);
      flywheelVolts = kV * flywheelSetpoint + flywheelPid.calculate(w, flywheelSetpoint) * 12.0;
      if (flywheelSetpoint <= 1.0) {
        flywheelVolts = 0.0;
      }
    }
    flywheel.setInputVoltage(MathUtil.clamp(flywheelVolts, 0.0, 12.0));
    flywheel.update(Constants.LOOP_PERIOD_SEC);

    if (hoodSetpoint != null) {
      hoodVolts = hoodPid.calculate(hood.getAngularPositionRad(), hoodSetpoint);
    }
    hood.setInputVoltage(MathUtil.clamp(hoodVolts, -12.0, 12.0));
    hood.update(Constants.LOOP_PERIOD_SEC);
    double hoodPos =
        MathUtil.clamp(hood.getAngularPositionRad(), 0.0, Units.degreesToRadians(MAX_LAUNCH_DEG - MIN_LAUNCH_DEG));
    if (hoodPos != hood.getAngularPositionRad()) {
      hood.setState(hoodPos, 0.0);
    }

    inputs.leftConnected = true;
    inputs.rightConnected = true;
    inputs.flywheelVelocityRadPerSec = flywheel.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = flywheelVolts;
    inputs.rightAppliedVolts = flywheelVolts;
    inputs.leftCurrentAmps = Math.abs(flywheel.getCurrentDrawAmps()) / 2.0;
    inputs.rightCurrentAmps = inputs.leftCurrentAmps;
    inputs.leftTempC = 30.0;
    inputs.rightTempC = 30.0;
    inputs.hoodConnected = true;
    inputs.hoodPositionRad = hood.getAngularPositionRad();
    inputs.hoodVelocityRadPerSec = hood.getAngularVelocityRadPerSec();
    inputs.hoodAppliedVolts = hoodVolts;
    inputs.hoodCurrentAmps = Math.abs(hood.getCurrentDrawAmps());
  }

  @Override
  public void setFlywheelVelocity(double radPerSec) {
    flywheelSetpoint = radPerSec;
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelSetpoint = null;
    flywheelVolts = volts;
  }

  @Override
  public void setHoodPosition(double liftRad) {
    hoodSetpoint = liftRad;
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodSetpoint = null;
    hoodVolts = volts;
  }

  @Override
  public void resetHoodPosition(double liftRad) {
    hood.setState(liftRad, 0.0);
  }
}
