package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.Motors;

/**
 * Sim: rotor and uptake as DC motor models. The FUEL count is kept by the superstructure's
 * {@code FuelSim}; this IO only reports the sensors that sim exposes through {@link #setSensors}.
 */
public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim rotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Motors.krakenX60Foc(1), 0.6, ROTOR_RATIO),
          Motors.krakenX60Foc(1));
  private final DCMotorSim uptake =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Motors.krakenX44(1), 0.0008, UPTAKE_RATIO),
          Motors.krakenX44(1));
  private final PIDController rotorPid = new PIDController(2.0, 0.0, 0.0);
  private final PIDController uptakePid = new PIDController(0.3, 0.0, 0.0);

  private Double rotorSetpointRps = null;
  private Double uptakeSetpointRps = null;
  private double rotorVolts = 0.0;
  private double uptakeVolts = 0.0;
  private boolean uptakeBall = false;
  private boolean hopperFull = false;

  /** Called by the FUEL simulation each loop. */
  public void setSensors(boolean ballAtUptake, boolean hopperIsFull) {
    uptakeBall = ballAtUptake;
    hopperFull = hopperIsFull;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    double rotorRps = rotor.getAngularVelocityRadPerSec() / (2 * Math.PI);
    double uptakeRps = uptake.getAngularVelocityRadPerSec() / (2 * Math.PI);
    if (rotorSetpointRps != null) {
      rotorVolts = 1.24 * rotorSetpointRps + rotorPid.calculate(rotorRps, rotorSetpointRps);
    }
    if (uptakeSetpointRps != null) {
      uptakeVolts = 0.0956 * uptakeSetpointRps + uptakePid.calculate(uptakeRps, uptakeSetpointRps);
    }
    rotor.setInputVoltage(MathUtil.clamp(rotorVolts, -12.0, 12.0));
    uptake.setInputVoltage(MathUtil.clamp(uptakeVolts, -12.0, 12.0));
    rotor.update(Constants.LOOP_PERIOD_SEC);
    uptake.update(Constants.LOOP_PERIOD_SEC);

    inputs.rotorConnected = true;
    inputs.rotorPositionRot = rotor.getAngularPositionRotations();
    inputs.rotorVelocityRps = rotorRps;
    inputs.rotorAppliedVolts = rotorVolts;
    inputs.rotorCurrentAmps = Math.abs(rotor.getCurrentDrawAmps());
    inputs.uptakeConnected = true;
    inputs.uptakeVelocityRps = uptakeRps;
    inputs.uptakeAppliedVolts = uptakeVolts;
    inputs.uptakeCurrentAmps = Math.abs(uptake.getCurrentDrawAmps());
    inputs.uptakeSensorConnected = true;
    inputs.uptakeBallDetected = uptakeBall;
    inputs.uptakeSensorDistanceM = uptakeBall ? 0.03 : 0.5;
    inputs.hopperSensorConnected = true;
    inputs.hopperFullDetected = hopperFull;
    inputs.hopperSensorDistanceM = hopperFull ? 0.1 : 0.6;
  }

  @Override
  public void setRotorVelocity(double rps) {
    rotorSetpointRps = rps;
  }

  @Override
  public void setRotorVoltage(double volts) {
    rotorSetpointRps = null;
    rotorVolts = volts;
  }

  @Override
  public void setUptakeVelocity(double rps) {
    uptakeSetpointRps = rps;
  }

  @Override
  public void setUptakeVoltage(double volts) {
    uptakeSetpointRps = null;
    uptakeVolts = volts;
  }
}
