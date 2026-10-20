package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Over-bumper intake. The arm rotates 127 deg from deployed (rollers on the carpet, 10.9 in past
 * the frame) to stowed (vertical, inside the frame perimeter and under 21.75 in for the TRENCH).
 * While deployed the deploy motor runs a low stator limit so a hit folds the arm instead of
 * bending it, and the superstructure re-deploys automatically.
 */
public class Intake extends SubsystemBase {
  public enum Goal {
    STOW,
    DEPLOY_IDLE,
    INTAKE,
    EJECT,
    /** Drive up into the stow stop and re-zero (run once after a brownout or a hard hit). */
    HOME
  }

  private static final double KG_VOLTS = 0.12;
  private static final double DEPLOYED_FROM_HORIZONTAL = Units.degreesToRadians(-37.0);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert deployDisconnected = new Alert("Intake deploy motor disconnected", AlertType.kError);
  private final Alert rollerDisconnected = new Alert("Intake roller motor disconnected", AlertType.kError);
  private final Debouncer homeStall = new Debouncer(0.15);
  private final Debouncer jamDebouncer = new Debouncer(0.25);

  private Goal goal = Goal.STOW;
  private boolean deployedLimitActive = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    deployDisconnected.set(!inputs.deployConnected);
    rollerDisconnected.set(!inputs.rollerConnected);

    if (DriverStation.isDisabled()) {
      io.setDeployVoltage(0.0);
      io.setRollerVoltage(0.0);
      return;
    }

    double ff = KG_VOLTS * Math.cos(inputs.deployPositionRad + DEPLOYED_FROM_HORIZONTAL);
    switch (goal) {
      case STOW -> {
        setCompliant(false);
        io.setDeployPosition(STOWED_RAD, ff);
        io.setRollerVoltage(0.0);
      }
      case DEPLOY_IDLE -> {
        setCompliant(isDeployed());
        io.setDeployPosition(DEPLOYED_RAD, ff);
        io.setRollerVoltage(0.0);
      }
      case INTAKE -> {
        setCompliant(isDeployed());
        io.setDeployPosition(DEPLOYED_RAD, ff);
        // Reverse briefly if the rollers stall on a jammed FUEL
        boolean jammed =
            jamDebouncer.calculate(
                inputs.rollerCurrentAmps > 55.0 && Math.abs(inputs.rollerVelocityRadPerSec) < 20.0);
        io.setRollerVoltage(jammed ? EJECT_VOLTS : INTAKE_VOLTS);
      }
      case EJECT -> {
        setCompliant(isDeployed());
        io.setDeployPosition(DEPLOYED_RAD, ff);
        io.setRollerVoltage(EJECT_VOLTS);
      }
      case HOME -> {
        setCompliant(false);
        io.setRollerVoltage(0.0);
        io.setDeployVoltage(1.5);
        if (homeStall.calculate(
            inputs.deployCurrentAmps > 12.0 && Math.abs(inputs.deployVelocityRadPerSec) < 0.2)) {
          io.resetDeployPosition(STOWED_RAD);
          goal = Goal.STOW;
        }
      }
    }
    Logger.recordOutput("Intake/Goal", goal);
  }

  private void setCompliant(boolean compliant) {
    if (compliant != deployedLimitActive) {
      io.setDeployStatorLimit(compliant ? DEPLOY_HOLD_STATOR_A : 40.0);
      deployedLimitActive = compliant;
    }
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput(key = "Intake/Deployed")
  public boolean isDeployed() {
    return Math.abs(inputs.deployPositionRad - DEPLOYED_RAD) < AT_POSITION_TOL_RAD;
  }

  @AutoLogOutput(key = "Intake/Stowed")
  public boolean isStowed() {
    return Math.abs(inputs.deployPositionRad - STOWED_RAD) < AT_POSITION_TOL_RAD;
  }

  /** Roller surface speed (m/s), used by the superstructure to estimate intake rate. */
  public double rollerSurfaceSpeed() {
    return inputs.rollerVelocityRadPerSec * ROLLER_DIA_M / 2.0;
  }

  public double getArmAngleRad() {
    return inputs.deployPositionRad;
  }

  public Command goalCommand(Goal g) {
    return runOnce(() -> setGoal(g)).withName("Intake " + g);
  }
}
