// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    
    public int getId() { return 0; }
    
    public static final class DriveConstants {
        public static final double MAX_SPEED_MPS = 4.5;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI;
        public static final double DEADBAND = 0.05;
        public static final double ROTATION_SCALE = 0.5;
        public static final double FULL_SPEED_ROTATION_SCALE = 1.0;
        public static final double PRECISION_MULTIPLIER = 0.3;
        public static final double PRECISION_ROTATION_SCALE = 0.3;
        
        // Auto constants
        public static final double AUTO_MAX_SPEED_MPS = 4.0;
        public static final double AUTO_POSITION_TOLERANCE_M = 0.20;
        public static final double AUTO_ANGLE_TOLERANCE_DEG = 15.0;
    }
    
    public static final class FieldConstants {
        // Field dimensions (Rebuilt 2025) - converted from feet to meters
        // 54.269 ft = 16.54 m, 26.474 ft = 8.07 m
        public static final double FIELD_LENGTH = 16.54;
        public static final double FIELD_WIDTH = 8.07;
        // starting positions
        public static final Pose2d BLUE_ALLIANCE_START = new Pose2d(1.0, 1.0, new Rotation2d(0));
        public static final Pose2d RED_ALLIANCE_START = new Pose2d(15.54, 1.0, new Rotation2d(Math.PI));
        public static final Pose2d CENTER_START = new Pose2d(8.27, 4.035, new Rotation2d(0)); // Updated to actual field center
        
        // Reef structure (center of field)
        public static final double REEF_CENTER_X = 8.27;
        public static final double REEF_CENTER_Y = 4.035;
        public static final double REEF_RADIUS = 1.5;
        public static final double REEF_SAFE_DISTANCE = 2.5;
        
        // Safe zones (away from obstacles)
        public static final Pose2d BLUE_SAFE_ZONE = new Pose2d(2.0, 1.5, Rotation2d.kZero);
        public static final Pose2d RED_SAFE_ZONE = new Pose2d(FIELD_LENGTH - 2.0, FIELD_WIDTH - 1.5, Rotation2d.fromDegrees(180));
        
        // Coral stations (at field edges)
        public static final Pose2d BLUE_CORAL_STATION = new Pose2d(1.2, 7.0, Rotation2d.kZero);
        public static final Pose2d RED_CORAL_STATION = new Pose2d(FIELD_LENGTH - 1.2, 1.2, Rotation2d.fromDegrees(180));
        
        // Reef branches (scoring positions) - adjusted for correct field center
        public static final Pose2d REEF_FRONT = new Pose2d(8.27, 2.5, Rotation2d.fromDegrees(0));
        public static final Pose2d REEF_LEFT = new Pose2d(7.2, 5.0, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_RIGHT = new Pose2d(9.3, 5.0, Rotation2d.fromDegrees(60));
        
        // Processor station
        public static final Pose2d PROCESSOR = new Pose2d(13.5, 4.035, Rotation2d.fromDegrees(90));

    // Convenient teleop rest position near the hub
    public static final Pose2d HUB_REST_POSITION = new Pose2d(8.27, 4.5, Rotation2d.kZero);
        
        // Field boundaries (with safety margin)
        public static final double FIELD_MARGIN = 0.5; // meters from edge
        public static final double MIN_X = FIELD_MARGIN;
        public static final double MAX_X = FIELD_LENGTH - FIELD_MARGIN;
        public static final double MIN_Y = FIELD_MARGIN;
        public static final double MAX_Y = FIELD_WIDTH - FIELD_MARGIN;
    }
    
    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double TRIGGER_THRESHOLD = 0.5;
    }
}
