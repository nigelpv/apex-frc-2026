package frc.robot.subsystems.turret;

import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.Motors;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Motors.krakenX44(1), MOI, MOTOR_TO_TURRET),
          Motors.krakenX44(1));
  private final PIDController pid = new PIDController(14.0, 0.0, 0.35);
  private final double startRad;

  private boolean closedLoop = false;
  private double setpointRad = 0.0;
  private double setpointVel = 0.0;
  private double volts = 0.0;
  private double offsetRad = 0.0;

  /** @param startRad where the turret physically starts (the vernier finds it) */
  public TurretIOSim(double startRad) {
    this.startRad = startRad;
    sim.setState(startRad, 0.0);
    offsetRad = -startRad; // like a real Talon, the motor encoder reads 0 at boot
  }

  public TurretIOSim() {
    this(Units.degreesToRadians(-30.0));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double truePos = sim.getAngularPositionRad();
    double measured = truePos + offsetRad;
    if (closedLoop) {
      double kV = 12.0 / (Units.rotationsPerMinuteToRadiansPerSecond(7530.0) / MOTOR_TO_TURRET);
      volts = kV * setpointVel + pid.calculate(measured, setpointRad);
    }
    // soft limits
    if ((measured >= MAX_RAD && volts > 0) || (measured <= MIN_RAD && volts < 0)) {
      volts = 0.0;
    }
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(volts);
    sim.update(Constants.LOOP_PERIOD_SEC);

    inputs.motorConnected = true;
    inputs.positionRad = sim.getAngularPositionRad() + offsetRad;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    double turns = sim.getAngularPositionRad() / (2 * Math.PI);
    inputs.encAConnected = true;
    inputs.encBConnected = true;
    inputs.encARot = wrapHalf(turns * RING_TEETH / ENC_A_TEETH);
    inputs.encBRot = wrapHalf(turns * RING_TEETH / ENC_B_TEETH);
  }

  private static double wrapHalf(double x) {
    return x - Math.floor(x + 0.5);
  }

  @Override
  public void setPosition(double positionRad, double velocityRadPerSec, boolean slew) {
    closedLoop = true;
    setpointRad = positionRad;
    setpointVel = slew ? 0.0 : velocityRadPerSec;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    this.volts = volts;
  }

  @Override
  public void resetPosition(double positionRad) {
    // the motor encoder in sim starts at 0 rad regardless of the true angle (like the real Talon)
    offsetRad = positionRad - sim.getAngularPositionRad();
  }

  /** Where the sim turret physically started. */
  public double getStartRad() {
    return startRad;
  }
}
