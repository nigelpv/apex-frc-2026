package frc.robot.subsystems.turret;

import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.VernierAngle;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Turret yaw. The absolute angle comes from the two vernier CANcoders at boot (and is re-checked
 * whenever the robot is disabled); after that the 70:1 motor encoder is the feedback sensor.
 * Setpoints are wrapped into the +/-190 deg service-loop window choosing the equivalent angle
 * closest to where the turret already is.
 */
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final VernierAngle vernier =
      new VernierAngle(
          RING_TEETH,
          ENC_A_TEETH,
          ENC_B_TEETH,
          Units.radiansToRotations(MIN_RAD) - 0.03,
          Units.radiansToRotations(MAX_RAD) + 0.03);
  private final Alert notSeeded = new Alert("Turret not zeroed: vernier encoders disagree", AlertType.kError);
  private final Alert motorDisconnected = new Alert("Turret motor disconnected", AlertType.kError);

  private static final int STABLE_LOOPS = 10;

  private boolean seeded = false;
  private double lastVernierRad = Double.NaN;
  private int stableCount = 0;
  private double goalRad = 0.0;
  private double goalVelRadPerSec = 0.0;
  private boolean hasGoal = false;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    motorDisconnected.set(!inputs.motorConnected);

    // Seed (or re-check while disabled) from the vernier pair
    if (inputs.encAConnected && inputs.encBConnected && (!seeded || DriverStation.isDisabled())) {
      VernierAngle.Result r = vernier.solve(inputs.encARot, inputs.encBRot);
      Logger.recordOutput("Turret/VernierRad", Units.rotationsToRadians(r.turretTurns()));
      Logger.recordOutput("Turret/VernierResidual", r.residual());
      // only trust an answer that stays put for STABLE_LOOPS loops (a slipped encoder gear can
      // land on another valid-looking candidate for a single sample)
      double abs = Units.rotationsToRadians(r.turretTurns());
      if (r.valid() && Math.abs(abs - lastVernierRad) < Units.degreesToRadians(0.5)) {
        stableCount++;
      } else {
        stableCount = 0;
      }
      lastVernierRad = r.valid() ? abs : Double.NaN;
      if (stableCount >= STABLE_LOOPS
          && (!seeded || Math.abs(abs - inputs.positionRad) > Units.degreesToRadians(1.5))) {
        io.resetPosition(abs);
        seeded = true;
      }
    }
    notSeeded.set(!seeded);

    if (DriverStation.isDisabled() || !seeded || !hasGoal) {
      io.setVoltage(0.0);
      return;
    }
    double err = goalRad - inputs.positionRad;
    boolean slew = Math.abs(err) > Units.degreesToRadians(20.0);
    io.setPosition(goalRad, goalVelRadPerSec, slew);
    Logger.recordOutput("Turret/GoalRad", goalRad);
    Logger.recordOutput("Turret/GoalVelRadPerSec", goalVelRadPerSec);
    Logger.recordOutput("Turret/Slewing", slew);
  }

  /**
   * Aim at a robot-relative angle with a feedforward rate. The angle may be any value; it is
   * wrapped into the legal window picking the equivalent nearest the current position.
   */
  public void aim(double robotRelativeRad, double velRadPerSec) {
    goalRad = bestEquivalent(robotRelativeRad, inputs.positionRad);
    goalVelRadPerSec = velRadPerSec;
    hasGoal = true;
  }

  public void stop() {
    hasGoal = false;
  }

  /** Pick theta + 2 pi k inside [MIN, MAX] closest to {@code current}. */
  static double bestEquivalent(double theta, double current) {
    double best = Double.NaN;
    double bestDist = Double.POSITIVE_INFINITY;
    for (int k = -2; k <= 2; k++) {
      double c = theta + 2.0 * Math.PI * k;
      if (c < MIN_RAD || c > MAX_RAD) {
        continue;
      }
      double d = Math.abs(c - current);
      if (d < bestDist) {
        bestDist = d;
        best = c;
      }
    }
    return Double.isNaN(best) ? MathUtil.clamp(theta, MIN_RAD, MAX_RAD) : best;
  }

  @AutoLogOutput(key = "Turret/AtGoal")
  public boolean atGoal() {
    return seeded
        && hasGoal
        && Math.abs(goalRad - inputs.positionRad) < AIM_TOL_RAD
        && Math.abs(goalVelRadPerSec - inputs.velocityRadPerSec) < Units.degreesToRadians(30.0);
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public boolean isSeeded() {
    return seeded;
  }
}
