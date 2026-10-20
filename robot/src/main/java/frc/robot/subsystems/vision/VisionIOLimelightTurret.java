package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.TurretConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Limelight 4 riding on the turret. Before every read the camera's robot-space pose is pushed to
 * the Limelight ("camerapose_robotspace_set") from the measured turret angle so MegaTag 2 solves
 * stay correct while the turret moves. Frames taken while the turret slews fast are discarded.
 */
public class VisionIOLimelightTurret implements VisionIO {
  private final VisionIOLimelight delegate;
  private final DoubleArrayPublisher cameraPosePublisher;
  private final DoubleSupplier turretAngleRad;
  private final DoubleSupplier turretRateRadPerSec;

  public VisionIOLimelightTurret(
      String name,
      Supplier<Rotation2d> robotRotation,
      DoubleSupplier turretAngleRad,
      DoubleSupplier turretRateRadPerSec) {
    delegate = new VisionIOLimelight(name, robotRotation);
    cameraPosePublisher =
        NetworkTableInstance.getDefault()
            .getTable(name)
            .getDoubleArrayTopic("camerapose_robotspace_set")
            .publish();
    this.turretAngleRad = turretAngleRad;
    this.turretRateRadPerSec = turretRateRadPerSec;
  }

  /** Robot-to-camera transform for a turret angle. */
  public static Transform3d robotToCamera(double turretRad) {
    Translation3d axis =
        new Translation3d(
            TurretConstants.AXIS_IN_ROBOT.getX(), TurretConstants.AXIS_IN_ROBOT.getY(), 0.0);
    Rotation3d yaw = new Rotation3d(0.0, 0.0, turretRad);
    Translation3d cam = turretToCamera0.getTranslation().rotateBy(yaw).plus(axis);
    Rotation3d rot = turretToCamera0.getRotation().plus(yaw);
    return new Transform3d(cam, rot);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Transform3d t = robotToCamera(turretAngleRad.getAsDouble());
    // Limelight convention: forward, side, up (m), roll, pitch, yaw (deg); pitch positive = up
    cameraPosePublisher.accept(
        new double[] {
          t.getX(),
          t.getY(),
          t.getZ(),
          Math.toDegrees(t.getRotation().getX()),
          -Math.toDegrees(t.getRotation().getY()),
          Math.toDegrees(t.getRotation().getZ())
        });
    delegate.updateInputs(inputs);
    if (Math.abs(turretRateRadPerSec.getAsDouble()) > maxTurretRateForVision) {
      inputs.poseObservations = new PoseObservation[0];
    }
  }
}
