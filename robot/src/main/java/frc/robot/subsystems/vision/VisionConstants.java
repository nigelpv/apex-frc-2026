// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on the Limelights
  public static String camera0Name = "limelight-turret"; // on the turret, looks where we shoot
  public static String camera1Name = "limelight-rl"; // rear-left deck corner
  public static String camera2Name = "limelight-rr"; // rear-right deck corner

  // Robot to camera transforms (from CAD). The fixed cameras are entered in the Limelight web UI;
  // the turret camera pose is published every loop by VisionIOLimelightTurret.
  public static Transform3d turretToCamera0 =
      new Transform3d(0.0583, 0.1416, 0.4735, new Rotation3d(0.0, Math.toRadians(-20.0), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          -0.307, 0.307, 0.361, new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(145.0)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.307, -0.307, 0.361, new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(-145.0)));

  /** Drop turret-camera frames while the turret slews faster than this (motion blur, pose lag). */
  public static double maxTurretRateForVision = 2.0; // rad/s

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.8, // Turret camera: closest to the HUB tags, trusted most
        1.0, // Rear left
        1.0 // Rear right
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
