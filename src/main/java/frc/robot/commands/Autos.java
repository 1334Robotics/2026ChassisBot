package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)),
        driveForTime(drive, 0.4, 0.0, 0.0, 2.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("Simple Forward Auto");
  }

  /**
   * Drive forward then backward.
   */
  public static Command forwardAndBackAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)),
        driveForTime(drive, 0.5, 0.0, 0.0, 2.0),
        Commands.waitSeconds(0.5),
        driveForTime(drive, -0.5, 0.0, 0.0, 2.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("Forward and Back Auto");
  }

  // ==================== PATTERN AUTOS ====================

  /**
   * Drive in a square pattern.
   */
  public static Command squarePathAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.CENTER_START)),
        // Side 1: Forward
        driveForTime(drive, 0.4, 0.0, 0.0, 1.5),
        Commands.waitSeconds(0.3),
        // Side 2: Strafe right
        driveForTime(drive, 0.0, -0.4, 0.0, 1.5),
        Commands.waitSeconds(0.3),
        // Side 3: Backward
        driveForTime(drive, -0.4, 0.0, 0.0, 1.5),
        Commands.waitSeconds(0.3),
        // Side 4: Strafe left
        driveForTime(drive, 0.0, 0.4, 0.0, 1.5),
        Commands.runOnce(() -> drive.stop())
    ).withName("Square Path Auto");
  }

  /**
   * Drive in a figure 8 pattern.
   */
  public static Command figureEightAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.CENTER_START)),
        // First circle (turn left while moving forward)
        driveForTime(drive, 0.3, 0.0, 0.4, 3.0),
        // Second circle (turn right while moving forward)
        driveForTime(drive, 0.3, 0.0, -0.4, 3.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("Figure 8 Auto");
  }

  /**
   * Spin in place autonomous.
   */
  public static Command spinInPlaceAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.CENTER_START)),
        // Spin 360 degrees clockwise
        driveForTime(drive, 0.0, 0.0, 0.5, 2.0),
        Commands.waitSeconds(0.5),
        // Spin 360 degrees counter-clockwise
        driveForTime(drive, 0.0, 0.0, -0.5, 2.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("Spin in Place Auto");
  }

  /**
   * S-Curve path autonomous.
   */
  public static Command sCurveAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)),
        // Curve right while moving forward
        driveForTime(drive, 0.4, 0.0, -0.3, 2.0),
        // Curve left while moving forward
        driveForTime(drive, 0.4, 0.0, 0.3, 2.0),
        // Curve right while moving forward
        driveForTime(drive, 0.4, 0.0, -0.3, 2.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("S-Curve Auto");
  }

  // ==================== COMPETITION AUTOS ====================

  /**
   * Leave the starting zone and stop (simple mobility auto).
   */
  public static Command mobilityAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)),
        // Drive forward to leave starting zone
        driveForTime(drive, 0.5, 0.0, 0.0, 3.0),
        Commands.runOnce(() -> drive.stop()),
        Commands.runOnce(() -> drive.lock())
    ).withName("Mobility Auto");
  }

  /**
   * Drive forward, turn around, and come back.
   */
  public static Command outAndBackAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)),
        // Drive forward
        driveForTime(drive, 0.5, 0.0, 0.0, 2.5),
        Commands.waitSeconds(0.3),
        // Turn 180 degrees
        driveForTime(drive, 0.0, 0.0, 0.5, 2.0),
        Commands.waitSeconds(0.3),
        // Drive back to start
        driveForTime(drive, 0.5, 0.0, 0.0, 2.5),
        Commands.runOnce(() -> drive.stop())
    ).withName("Out and Back Auto");
  }

  /**
   * Strafe test - move sideways.
   */
  public static Command strafeTestAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.CENTER_START)),
        // Strafe left
        driveForTime(drive, 0.0, 0.4, 0.0, 2.0),
        Commands.waitSeconds(0.5),
        // Strafe right
        driveForTime(drive, 0.0, -0.4, 0.0, 2.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("Strafe Test Auto");
  }

  /**
   * Diagonal drive test.
   */
  public static Command diagonalDriveAuto(DriveSubsystem drive) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)),
        // Drive diagonally forward-right
        driveForTime(drive, 0.4, -0.4, 0.0, 2.0),
        Commands.waitSeconds(0.3),
        // Drive diagonally forward-left
        driveForTime(drive, 0.4, 0.4, 0.0, 2.0),
        Commands.waitSeconds(0.3),
        // Drive diagonally backward-left
        driveForTime(drive, -0.4, 0.4, 0.0, 2.0),
        Commands.waitSeconds(0.3),
        // Drive diagonally backward-right
        driveForTime(drive, -0.4, -0.4, 0.0, 2.0),
        Commands.runOnce(() -> drive.stop())
    ).withName("Diagonal Drive Auto");
  }

  // ==================== HELPER METHODS ====================

  /**
   * Helper method to create a timed drive command.
   * 
   * @param drive DriveSubsystem instance
   * @param xSpeed Forward/backward speed (-1 to 1)
   * @param ySpeed Left/right strafe speed (-1 to 1)
   * @param rotSpeed Rotation speed (-1 to 1)
   * @param seconds Duration in seconds
   * @return Command that drives for the specified time
   */
  public static Command driveForTime(DriveSubsystem drive, double xSpeed, double ySpeed, double rotSpeed, double seconds) {
    return drive.driveCommand(
        () -> xSpeed,
        () -> ySpeed,
        () -> rotSpeed,
        () -> 0.0
    ).withTimeout(seconds);
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
