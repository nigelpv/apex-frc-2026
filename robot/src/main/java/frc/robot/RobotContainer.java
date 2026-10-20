package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.HubShift;
import frc.robot.lib.ShotSolver;
import frc.robot.lib.ShotTables;
import frc.robot.sim.FuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOLimelightTurret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Builds the robot: subsystems (real, sim or replay IO), the superstructure that runs scoring on
 * its own, the driver / operator bindings and the AUTO chooser.
 *
 * <p>Driver (Xbox 0): left stick drive, right stick turn (field relative). Right trigger intake,
 * left trigger force-shoot (ignores the HUB schedule, never G407), right bumper pass, left bumper
 * hold fire, Y eject, X X-lock, B re-zero heading, D-pad down drive home through the nearest
 * TRENCH, start re-home the intake arm. Rumble: our HUB turns active within 3 s while loaded.
 * Near a TRENCH, with the right stick centred, the heading squares itself to the lane.
 *
 * <p>Operator (Xbox 1): A toggle auto-pass, B eject, D-pad up/down shot speed trim +/-1 %,
 * back + start mark the robot loaded.
 */
public class RobotContainer {
  private static final double RELEASE_LATENCY_SEC = 0.08;

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;
  private final Superstructure superstructure;
  private final FuelSim fuelSim;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final LoggedDashboardChooser<Command> autoChooser;

  private HubShift.AutoWinner autoWinner = HubShift.AutoWinner.UNKNOWN;

  public RobotContainer() {
    IndexerIOSim indexerSim = null;
    ShooterIOSim shooterSim = null;
    switch (Constants.currentMode) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelightTurret(
                    camera0Name, drive::getRotation, turret::getPositionRad, turret::getVelocityRadPerSec),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation));
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim());
        indexerSim = new IndexerIOSim();
        indexer = new Indexer(indexerSim);
        turret = new Turret(new TurretIOSim());
        shooterSim = new ShooterIOSim();
        shooter = new Shooter(shooterSim);
        // no camera sim: odometry is perfect in sim
        vision = new Vision(drive::addVisionMeasurement);
      }
      default -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        turret = new Turret(new TurretIO() {});
        shooter = new Shooter(new ShooterIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
      }
    }

    double ax = TurretConstants.AXIS_IN_ROBOT.getX();
    double ay = TurretConstants.AXIS_IN_ROBOT.getY();
    superstructure =
        new Superstructure(
            drive,
            intake,
            indexer,
            turret,
            shooter,
            new ShotSolver(ShotTables.hub(), ax, ay, RELEASE_LATENCY_SEC),
            new ShotSolver(ShotTables.pass(), ax, ay, RELEASE_LATENCY_SEC),
            () -> autoWinner);

    fuelSim =
        Constants.currentMode == Constants.Mode.SIM
            ? new FuelSim(drive, intake, indexer, turret, shooter, indexerSim, shooterSim, () -> autoWinner)
            : null;
    if (fuelSim != null) {
      drive.setPose(FieldConstants.allianceFlip(Autos.RIGHT_START));
    }

    // AUTO routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Right Double Sweep", Autos.doubleSweep(drive, superstructure, false));
    autoChooser.addOption("Left Double Sweep", Autos.doubleSweep(drive, superstructure, true));
    autoChooser.addOption(
        "Shoot Preload (right start)", Autos.shootPreload(drive, superstructure, Autos.RIGHT_START));
    autoChooser.addOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDriveTrenchAssist(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> Superstructure.nearTrench(drive.getPose(), drive.getFieldVelocity())));

    // ---------------- driver
    driver.rightTrigger(0.3).whileTrue(superstructure.intakeWhileHeld());
    driver.leftTrigger(0.3).whileTrue(superstructure.forceShootWhileHeld());
    driver.rightBumper().whileTrue(superstructure.passWhileHeld());
    driver.leftBumper().whileTrue(superstructure.holdFireWhileHeld());
    driver.y().whileTrue(superstructure.ejectWhileHeld());
    driver.x().whileTrue(Commands.run(drive::stopWithX, drive));
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                FieldConstants.isRed() ? Rotation2d.kPi : Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driver.povDown().whileTrue(Autos.returnThroughTrench(drive));
    driver.start().onTrue(intake.goalCommand(Intake.Goal.HOME));

    // our HUB becomes active soon and we are loaded: rumble so the driver heads for the zone
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && superstructure.isHoldingFuel()
                    && hubActivatingSoon(3.0))
        .whileTrue(
            Commands.startEnd(
                () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.6),
                () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ---------------- operator
    operator.a().onTrue(Commands.runOnce(superstructure::toggleAutoPass));
    operator.b().whileTrue(superstructure.ejectWhileHeld());
    operator.povUp().onTrue(Commands.runOnce(() -> superstructure.adjustSpeedTrim(0.01)));
    operator.povDown().onTrue(Commands.runOnce(() -> superstructure.adjustSpeedTrim(-0.01)));
    operator
        .back()
        .and(operator.start())
        .onTrue(Commands.runOnce(() -> superstructure.setHoldingFuel(true)).ignoringDisable(true));
  }

  private boolean hubActivatingSoon(double withinSec) {
    double t = DriverStation.getMatchTime();
    if (t < 0 || HubShift.ourHubActive(t, autoWinner)) {
      return false;
    }
    return HubShift.secondsToNextChange(t, autoWinner) < withinSec;
  }

  /** Called every loop from Robot.robotPeriodic (before the scheduler). */
  public void updateGameData() {
    HubShift.AutoWinner w =
        HubShift.parseGameData(DriverStation.getGameSpecificMessage(), FieldConstants.isRed());
    if (w != HubShift.AutoWinner.UNKNOWN || DriverStation.isDisabled()) {
      autoWinner = w;
    }
    Logger.recordOutput("Match/AutoWinner", autoWinner);
    double t = DriverStation.getMatchTime();
    Logger.recordOutput("Match/Time", t);
    if (DriverStation.isTeleop() && t >= 0) {
      Logger.recordOutput("Match/OurHubActive", HubShift.ourHubActive(t, autoWinner));
      Logger.recordOutput("Match/SecondsToHubChange", HubShift.secondsToNextChange(t, autoWinner));
    }
  }

  public void autonomousInit() {
    superstructure.setHoldingFuel(true);
    if (fuelSim != null) {
      fuelSim.reset();
    }
  }

  public void simulationPeriodic() {
    if (fuelSim != null) {
      fuelSim.update();
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
