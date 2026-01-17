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
 */
public final class Constants {
  
  /**
   * Drive subsystem constants.
   */
  public static final class DriveConstants {
    // Maximum speeds
    public static final double MAX_SPEED_MPS = 4.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 1.5; // radians per second (270 deg/s)
    
    // Joystick deadband
    public static final double DEADBAND = 0.1;
    
    // Speed control
    public static final double DEFAULT_SPEED_MULTIPLIER = 0.5;
    public static final double SPEED_INCREMENT = 0.1;
    public static final double MIN_SPEED_MULTIPLIER = 0.1;
    public static final double MAX_SPEED_MULTIPLIER = 1.0;
    
    // Rotation scaling
    public static final double ROTATION_SCALE = 0.6;
    public static final double PRECISION_ROTATION_SCALE = 0.1;
    public static final double FULL_SPEED_ROTATION_SCALE = 0.5;
    
    // Precision mode multiplier
    public static final double PRECISION_MULTIPLIER = 0.2;
  }

  /**
   * Vision constants.
   */
  public static final class VisionConstants {
    // Camera configuration
    public static final double CAMERA_HEIGHT_METERS = 0.5;
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 30.0;
    public static final double TARGET_HEIGHT_METERS = 2.5;
    
    // Pipeline indices
    public static final int APRILTAG_PIPELINE = 0;
    public static final int REFLECTIVE_TAPE_PIPELINE = 1;
    
    // Alignment
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
    
    // Limelight configuration
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double MIN_TARGET_AREA = 0.1;
    public static final double MAX_POSE_AMBIGUITY = 0.7;
    public static final double CONNECTION_TIMEOUT_SECONDS = 0.5;
  }

  /**
   * Field dimensions and starting positions.
   */
  public static final class FieldConstants {
    // Field dimensions in meters
    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.02;
    
    // Starting positions (Blue alliance origin)
    public static final Pose2d BLUE_ALLIANCE_START = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_ALLIANCE_START = new Pose2d(FIELD_LENGTH - 1.5, 5.5, Rotation2d.fromDegrees(180));
    public static final Pose2d CENTER_START = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(0));
    
    // Field margins for pose validation
    public static final double POSE_MARGIN = 0.5;
  }

  /**
   * Controller constants.
   */
  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    
    // Trigger thresholds
    public static final double TRIGGER_THRESHOLD = 0.5;
  }

  /**
   * Autonomous constants.
   */
  public static final class AutoConstants {
    // Default auto speeds (slower for safety)
    public static final double AUTO_DRIVE_SPEED = 1.5; // m/s
    public static final double AUTO_ROTATION_SPEED = Math.PI / 2; // rad/s (90 deg/s)
    
    // Timing
    public static final double DEFAULT_DRIVE_TIME = 2.0; // seconds
    public static final double DEFAULT_ROTATE_TIME = 1.0; // seconds
  }
}
