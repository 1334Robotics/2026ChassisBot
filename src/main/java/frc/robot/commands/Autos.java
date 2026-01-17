package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Utility class containing all autonomous routines.
 */
public final class Autos {

  // ==================== SIMPLE AUTOS ====================

  /**
   * Simple drive-forward autonomous (2 seconds at 40% speed)
   */
  public static Command simpleForwardAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.BLUE_ALLIANCE_START),
        timedDrive(drive, 0.4, 0.0, 0.0, 2.0),
        stopDrive(drive)
    ).withName("Simple Forward Auto");
  }

  /**
   * Drive forward then backward.
   */
  public static Command forwardAndBackAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.BLUE_ALLIANCE_START),
        timedDrive(drive, 0.5, 0.0, 0.0, 2.0),
        Commands.waitSeconds(0.5),
        timedDrive(drive, -0.5, 0.0, 0.0, 2.0),
        stopDrive(drive)
    ).withName("Forward and Back Auto");
  }

  // ==================== PATTERN AUTOS ====================

  /**
   * Drive in a square pattern.
   */
  public static Command squarePathAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.CENTER_START),
        // Side 1: Forward
        timedDrive(drive, 0.4, 0.0, 0.0, 1.5),
        Commands.waitSeconds(0.3),
        // Side 2: Strafe right
        timedDrive(drive, 0.0, -0.4, 0.0, 1.5),
        Commands.waitSeconds(0.3),
        // Side 3: Backward
        timedDrive(drive, -0.4, 0.0, 0.0, 1.5),
        Commands.waitSeconds(0.3),
        // Side 4: Strafe left
        timedDrive(drive, 0.0, 0.4, 0.0, 1.5),
        stopDrive(drive)
    ).withName("Square Path Auto");
  }

  /**
   * Drive in a figure 8 pattern.
   */
  public static Command figureEightAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.CENTER_START),
        // First circle (turn left while moving forward)
        timedDrive(drive, 0.3, 0.0, 0.4, 3.0),
        // Second circle (turn right while moving forward)
        timedDrive(drive, 0.3, 0.0, -0.4, 3.0),
        stopDrive(drive)
    ).withName("Figure 8 Auto");
  }

  /**
   * Spin in place autonomous.
   */
  public static Command spinInPlaceAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.CENTER_START),
        // Spin clockwise
        timedDrive(drive, 0.0, 0.0, 0.5, 2.0),
        Commands.waitSeconds(0.5),
        // Spin counter-clockwise
        timedDrive(drive, 0.0, 0.0, -0.5, 2.0),
        stopDrive(drive)
    ).withName("Spin in Place Auto");
  }

  /**
   * S-Curve path autonomous.
   */
  public static Command sCurveAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.BLUE_ALLIANCE_START),
        // Curve right while moving forward
        timedDrive(drive, 0.4, 0.0, -0.3, 2.0),
        // Curve left while moving forward
        timedDrive(drive, 0.4, 0.0, 0.3, 2.0),
        // Curve right while moving forward
        timedDrive(drive, 0.4, 0.0, -0.3, 2.0),
        stopDrive(drive)
    ).withName("S-Curve Auto");
  }

  // ==================== COMPETITION AUTOS ====================

  /**
   * Leave the starting zone and stop (simple mobility auto).
   */
  public static Command mobilityAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.BLUE_ALLIANCE_START),
        timedDrive(drive, 0.5, 0.0, 0.0, 3.0),
        stopDrive(drive),
        Commands.runOnce(() -> drive.lock())
    ).withName("Mobility Auto");
  }

  /**
   * Drive forward, turn around, and come back.
   */
  public static Command outAndBackAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.BLUE_ALLIANCE_START),
        // Drive forward
        timedDrive(drive, 0.5, 0.0, 0.0, 2.5),
        Commands.waitSeconds(0.3),
        // Turn 180 degrees
        timedDrive(drive, 0.0, 0.0, 0.5, 2.0),
        Commands.waitSeconds(0.3),
        // Drive back to start
        timedDrive(drive, 0.5, 0.0, 0.0, 2.5),
        stopDrive(drive)
    ).withName("Out and Back Auto");
  }

  /**
   * Strafe test - move sideways.
   */
  public static Command strafeTestAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.CENTER_START),
        // Strafe left
        timedDrive(drive, 0.0, 0.4, 0.0, 2.0),
        Commands.waitSeconds(0.5),
        // Strafe right
        timedDrive(drive, 0.0, -0.4, 0.0, 2.0),
        stopDrive(drive)
    ).withName("Strafe Test Auto");
  }

  /**
   * Diagonal drive test.
   */
  public static Command diagonalDriveAuto(DriveSubsystem drive) {
    return Commands.sequence(
        resetPose(drive, Constants.FieldConstants.BLUE_ALLIANCE_START),
        // Drive diagonally forward-right
        timedDrive(drive, 0.4, -0.4, 0.0, 2.0),
        Commands.waitSeconds(0.3),
        // Drive diagonally forward-left
        timedDrive(drive, 0.4, 0.4, 0.0, 2.0),
        Commands.waitSeconds(0.3),
        // Drive diagonally backward-left
        timedDrive(drive, -0.4, 0.4, 0.0, 2.0),
        Commands.waitSeconds(0.3),
        // Drive diagonally backward-right
        timedDrive(drive, -0.4, -0.4, 0.0, 2.0),
        stopDrive(drive)
    ).withName("Diagonal Drive Auto");
  }

  // ==================== HELPER METHODS ====================

  /**
   * Creates a command that drives for a specified time using FunctionalCommand.
   * This properly handles the subsystem requirement.
   */
  private static Command timedDrive(DriveSubsystem drive, double xSpeed, double ySpeed, double rotSpeed, double seconds) {
    final double maxSpeed = 3.0; // m/s
    final double maxAngular = Math.PI; // rad/s
    
    return new FunctionalCommand(
        // Init - log start
        () -> SmartDashboard.putString("Auto/Current Step", 
            String.format("Drive: X=%.1f Y=%.1f R=%.1f for %.1fs", xSpeed, ySpeed, rotSpeed, seconds)),
        // Execute - drive the robot
        () -> {
            ChassisSpeeds speeds = new ChassisSpeeds(
                xSpeed * maxSpeed,
                ySpeed * maxSpeed,
                rotSpeed * maxAngular
            );
            drive.driveFieldOriented(speeds);
        },
        // End - stop the robot
        (interrupted) -> {
            drive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Auto/Current Step", interrupted ? "Interrupted" : "Step Complete");
        },
        // IsFinished - never (timeout handles this)
        () -> false,
        // Requirements
        drive
    ).withTimeout(seconds);
  }

  /**
   * Creates a command to reset the robot pose.
   */
  private static Command resetPose(DriveSubsystem drive, edu.wpi.first.math.geometry.Pose2d pose) {
    return Commands.runOnce(() -> {
        drive.resetOdometry(pose);
        SmartDashboard.putString("Auto/Current Step", "Reset Pose");
    });
  }

  /**
   * Creates a command to stop the drive.
   */
  private static Command stopDrive(DriveSubsystem drive) {
    return Commands.runOnce(() -> {
        drive.stop();
        SmartDashboard.putString("Auto/Current Step", "Stopped");
    }, drive);
  }

  /**
   * Do nothing autonomous.
   */
  public static Command doNothingAuto() {
    return Commands.none().withName("Do Nothing");
  }

  // Private constructor to prevent instantiation
  private Autos() {
    throw new UnsupportedOperationException("Utility class - do not instantiate");
  }
}
