package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.util.Motors;

/** Physics sim: single-jointed arm for the deploy, flywheel model for the roller train. */
public class IntakeIOSim implements IntakeIO {
  /** Deployed arm sits 37 deg below horizontal; the sim works in "from horizontal" angles. */
  private static final double DEPLOYED_FROM_HORIZONTAL = Units.degreesToRadians(-37.0);

  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          Motors.krakenX44(1),
          DEPLOY_RATIO,
          ARM_MOI,
          ARM_LENGTH_M,
          DEPLOYED_FROM_HORIZONTAL,
          DEPLOYED_FROM_HORIZONTAL + STOWED_RAD,
          true,
          DEPLOYED_FROM_HORIZONTAL + STOWED_RAD);
  private final FlywheelSim roller =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(Motors.krakenX44(1), 0.0015, ROLLER_RATIO),
          Motors.krakenX44(1));
  private final ProfiledPIDController armController =
      new ProfiledPIDController(
          28.0, 0.0, 1.0, new TrapezoidProfile.Constraints(Units.rotationsToRadians(1.2), 50.0));

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double deployVolts = 0.0;
  private double rollerVolts = 0.0;
  private double offsetRad = 0.0;

  public IntakeIOSim() {
    armController.reset(STOWED_RAD);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double position = arm.getAngleRads() - DEPLOYED_FROM_HORIZONTAL + offsetRad;
    if (closedLoop) {
      deployVolts = armController.calculate(position) + ffVolts;
    }
    deployVolts = MathUtil.clamp(deployVolts, -12.0, 12.0);
    arm.setInputVoltage(deployVolts);
    arm.update(Constants.LOOP_PERIOD_SEC);
    roller.setInputVoltage(MathUtil.clamp(rollerVolts, -12.0, 12.0));
    roller.update(Constants.LOOP_PERIOD_SEC);

    inputs.deployConnected = true;
    inputs.deployPositionRad = arm.getAngleRads() - DEPLOYED_FROM_HORIZONTAL + offsetRad;
    inputs.deployVelocityRadPerSec = arm.getVelocityRadPerSec();
    inputs.deployAppliedVolts = deployVolts;
    inputs.deployCurrentAmps = Math.abs(arm.getCurrentDrawAmps());
    inputs.rollerConnected = true;
    inputs.rollerVelocityRadPerSec = roller.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerVolts;
    inputs.rollerCurrentAmps = Math.abs(roller.getCurrentDrawAmps());
  }

  @Override
  public void setDeployPosition(double positionRad, double feedforwardVolts) {
    if (!closedLoop) {
      armController.reset(arm.getAngleRads() - DEPLOYED_FROM_HORIZONTAL + offsetRad);
    }
    closedLoop = true;
    armController.setGoal(positionRad);
    ffVolts = feedforwardVolts;
  }

  @Override
  public void setDeployVoltage(double volts) {
    closedLoop = false;
    deployVolts = volts;
  }

  @Override
  public void resetDeployPosition(double positionRad) {
    offsetRad = positionRad - (arm.getAngleRads() - DEPLOYED_FROM_HORIZONTAL);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerVolts = volts;
  }
}
