package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel + hood. The superstructure asks for a ball exit speed and launch angle; this class
 * converts them to wheel speed (hooded single wheel: exit speed ~= efficiency x surface speed)
 * and hood lift. A TRENCH guard forces the hood onto its down stop regardless of the request.
 */
public class Shooter extends SubsystemBase {
  private static final double IDLE_RPM = 1800.0;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert flywheelDisconnected = new Alert("Flywheel motor disconnected", AlertType.kError);
  private final Alert hoodDisconnected = new Alert("Hood motor disconnected", AlertType.kError);
  private final Alert hot = new Alert("Flywheel motors above 80 C", AlertType.kWarning);

  private double goalRpm = 0.0;
  private double goalLaunchDeg = MIN_LAUNCH_DEG;
  private boolean trenchGuard = false;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    flywheelDisconnected.set(!inputs.leftConnected || !inputs.rightConnected);
    hoodDisconnected.set(!inputs.hoodConnected);
    hot.set(Math.max(inputs.leftTempC, inputs.rightTempC) > 80.0);

    if (DriverStation.isDisabled()) {
      io.setFlywheelVoltage(0.0);
      io.setHoodVoltage(0.0);
      return;
    }
    if (goalRpm <= 1.0) {
      io.setFlywheelVoltage(0.0);
    } else {
      io.setFlywheelVelocity(Units.rotationsPerMinuteToRadiansPerSecond(goalRpm));
    }
    double lift = trenchGuard ? 0.0 : Units.degreesToRadians(goalLaunchDeg - MIN_LAUNCH_DEG);
    io.setHoodPosition(lift);
    Logger.recordOutput("Shooter/GoalRpm", goalRpm);
    Logger.recordOutput("Shooter/GoalLaunchDeg", goalLaunchDeg);
    Logger.recordOutput("Shooter/TrenchGuard", trenchGuard);
  }

  /** Request a shot: exit speed (m/s) and launch angle (deg above horizontal). */
  public void setShot(double exitSpeedMps, double launchDeg) {
    double surface = exitSpeedMps / EXIT_EFFICIENCY;
    goalRpm = MathUtil.clamp(surface / (Math.PI * WHEEL_DIA_M) * 60.0, 0.0, MAX_RPM);
    goalLaunchDeg = MathUtil.clamp(launchDeg, MIN_LAUNCH_DEG, MAX_LAUNCH_DEG);
  }

  /** Keep the wheel spinning slowly between shots so spin-up takes ~0.3 s instead of ~0.7 s. */
  public void setIdle() {
    goalRpm = IDLE_RPM;
    goalLaunchDeg = MIN_LAUNCH_DEG;
  }

  public void stop() {
    goalRpm = 0.0;
    goalLaunchDeg = MIN_LAUNCH_DEG;
  }

  public void setTrenchGuard(boolean guard) {
    trenchGuard = guard;
  }

  @AutoLogOutput(key = "Shooter/FlywheelRpm")
  public double getRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.flywheelVelocityRadPerSec);
  }

  @AutoLogOutput(key = "Shooter/LaunchDeg")
  public double getLaunchDeg() {
    return MIN_LAUNCH_DEG + Units.radiansToDegrees(inputs.hoodPositionRad);
  }

  /** Actual exit speed the wheel would give right now (for logging / shot quality). */
  public double getExitSpeedMps() {
    return inputs.flywheelVelocityRadPerSec * WHEEL_DIA_M / 2.0 * EXIT_EFFICIENCY;
  }

  @AutoLogOutput(key = "Shooter/Ready")
  public boolean isReady() {
    return goalRpm > IDLE_RPM
        && Math.abs(getRpm() - goalRpm) < RPM_TOL
        && (trenchGuard
            ? getLaunchDeg() < MIN_LAUNCH_DEG + HOOD_TOL_DEG
            : Math.abs(getLaunchDeg() - goalLaunchDeg) < HOOD_TOL_DEG);
  }

  /** Hood is on its down stop (robot fits under the TRENCH). */
  @AutoLogOutput(key = "Shooter/HoodDown")
  public boolean isHoodDown() {
    return inputs.hoodPositionRad < Units.degreesToRadians(1.0);
  }

  public double getGoalRpm() {
    return goalRpm;
  }
}
