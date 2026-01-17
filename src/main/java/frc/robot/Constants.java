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
    
    public static final class DriveConstants {
        public static final double MAX_SPEED_MPS = 4.5;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI;
        public static final double DEADBAND = 0.05;
        public static final double ROTATION_SCALE = 0.5;
        public static final double FULL_SPEED_ROTATION_SCALE = 1.0;
        public static final double PRECISION_MULTIPLIER = 0.3;
        public static final double PRECISION_ROTATION_SCALE = 0.3;
    }
    
    public static final class FieldConstants {
        public static final double FIELD_LENGTH = 17.54;
        public static final double FIELD_WIDTH = 8.21;
        
        public static final Pose2d BLUE_ALLIANCE_START = new Pose2d(1.5, 4.105, Rotation2d.kZero);
        public static final Pose2d RED_ALLIANCE_START = new Pose2d(FIELD_LENGTH - 1.5, 4.105, Rotation2d.fromDegrees(180));
        public static final Pose2d CENTER_START = new Pose2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, Rotation2d.kZero);
        
        public static final Pose2d BLUE_SAFE_ZONE = new Pose2d(2.5, 1.5, Rotation2d.kZero);
        public static final Pose2d RED_SAFE_ZONE = new Pose2d(FIELD_LENGTH - 2.5, FIELD_WIDTH - 1.5, Rotation2d.fromDegrees(180));
        
        public static final Pose2d BLUE_CORAL_STATION = new Pose2d(1.2, 7.0, Rotation2d.kZero);
        public static final Pose2d RED_CORAL_STATION = new Pose2d(FIELD_LENGTH - 1.2, 1.21, Rotation2d.fromDegrees(180));
        
        public static final Pose2d REEF_FRONT = new Pose2d(8.77, 2.6, Rotation2d.kZero);
        public static final Pose2d REEF_LEFT = new Pose2d(7.57, 5.105, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_RIGHT = new Pose2d(10.0, 5.105, Rotation2d.fromDegrees(60));
        
        public static final Pose2d PROCESSOR = new Pose2d(14.0, 4.1, Rotation2d.fromDegrees(90));
    }
    
    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double TRIGGER_THRESHOLD = 0.5;
    }
}
