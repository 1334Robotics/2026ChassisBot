// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {
    public static final double NORMAL_SPEED_MULTIPLIER = 0.5;
    public static final double TURBO_SPEED_MULTIPLIER = 1.0;
    public static final double PRECISION_SPEED_MULTIPLIER = 0.2;
    public static final double ROTATION_SPEED_MULTIPLIER = 0.3;
    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  public static final class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = 0.5;
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 30.0;
    public static final double TARGET_HEIGHT_METERS = 2.5;
    public static final int APRILTAG_PIPELINE = 0;
    public static final int REFLECTIVE_TAPE_PIPELINE = 1;
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
  }

  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.21;
    public static final Pose2d BLUE_ALLIANCE_START = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_ALLIANCE_START = new Pose2d(15.0, 4.0, Rotation2d.fromDegrees(180));
    public static final Pose2d CENTER_START = new Pose2d(8.27, 4.1, Rotation2d.fromDegrees(0));
  }
}
