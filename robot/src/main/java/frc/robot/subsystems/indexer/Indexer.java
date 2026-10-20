package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Rotor serializer + uptake. The rotor turns FUEL dumped anywhere in the hopper into a single
 * file at the rear pocket under the turret axis; the uptake belts lift it through the turret
 * bearing into the flywheel. Five pockets at 4 rotor rev/s feed 20 FUEL/s.
 */
public class Indexer extends SubsystemBase {
  public enum Goal {
    IDLE,
    /** Slow rotor while intaking so FUEL settles into pockets and nothing bridges. */
    AGITATE,
    /** Bring the next FUEL up to the uptake sensor and hold it there (instant first shot). */
    STAGE,
    /** Full-rate feed into the shooter. */
    FEED,
    /** Reverse both stages to clear a jam or dump FUEL out of the intake. */
    REVERSE
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Debouncer rotorJam = new Debouncer(0.2);
  private final Alert rotorDisconnected = new Alert("Serializer motor disconnected", AlertType.kError);
  private final Alert uptakeDisconnected = new Alert("Uptake motor disconnected", AlertType.kError);
  private final Alert sensorDisconnected = new Alert("Uptake CANrange disconnected: feeding blind", AlertType.kWarning);

  private Goal goal = Goal.IDLE;
  private double unjamUntil = 0.0;
  private boolean lastBall = false;
  private int shotsCounted = 0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    rotorDisconnected.set(!inputs.rotorConnected);
    uptakeDisconnected.set(!inputs.uptakeConnected);
    sensorDisconnected.set(!inputs.uptakeSensorConnected);

    // count FUEL leaving the uptake (falling edge of the sensor while feeding)
    if (lastBall && !inputs.uptakeBallDetected && goal == Goal.FEED) {
      shotsCounted++;
    }
    lastBall = inputs.uptakeBallDetected;

    if (DriverStation.isDisabled()) {
      io.setRotorVoltage(0.0);
      io.setUptakeVoltage(0.0);
      return;
    }

    double now = Timer.getFPGATimestamp();
    boolean jam =
        rotorJam.calculate(
            goal != Goal.IDLE
                && goal != Goal.REVERSE
                && inputs.rotorCurrentAmps > 70.0
                && Math.abs(inputs.rotorVelocityRps) < 0.15);
    if (jam) {
      unjamUntil = now + 0.3;
    }
    if (now < unjamUntil) {
      io.setRotorVoltage(-6.0);
      io.setUptakeVoltage(goal == Goal.FEED ? 4.0 : 0.0);
      Logger.recordOutput("Indexer/Unjamming", true);
      return;
    }
    Logger.recordOutput("Indexer/Unjamming", false);

    double uptakeFeedRps = UPTAKE_FEED_MPS / (Math.PI * UPTAKE_PULLEY_PITCH_DIA_M);
    switch (goal) {
      case IDLE -> {
        io.setRotorVoltage(0.0);
        io.setUptakeVoltage(0.0);
      }
      case AGITATE -> {
        io.setRotorVelocity(ROTOR_AGITATE_RPS);
        io.setUptakeVoltage(0.0);
      }
      case STAGE -> {
        if (inputs.uptakeBallDetected) {
          io.setRotorVoltage(0.0);
          io.setUptakeVoltage(0.0);
        } else {
          io.setRotorVelocity(1.0);
          io.setUptakeVelocity(uptakeFeedRps * 0.25);
        }
      }
      case FEED -> {
        io.setRotorVelocity(ROTOR_FEED_RPS);
        io.setUptakeVelocity(uptakeFeedRps);
      }
      case REVERSE -> {
        io.setRotorVelocity(-1.5);
        io.setUptakeVoltage(-6.0);
      }
    }
    Logger.recordOutput("Indexer/Goal", goal);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput(key = "Indexer/BallStaged")
  public boolean isBallStaged() {
    return inputs.uptakeBallDetected;
  }

  @AutoLogOutput(key = "Indexer/HopperFull")
  public boolean isHopperFull() {
    return inputs.hopperFullDetected;
  }

  @AutoLogOutput(key = "Indexer/ShotsCounted")
  public int getShotsCounted() {
    return shotsCounted;
  }
}
