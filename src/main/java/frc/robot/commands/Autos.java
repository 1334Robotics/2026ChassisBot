package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Utility class containing autonomous routines.
 */
public final class Autos {

  /**
   * Do nothing autonomous - robot stays still.
   */
  public static Command doNothingAuto() {
    return Commands.none().withName("Do Nothing");
  }

  /**
   * Drive forward autonomous routine.
   * Robot drives forward at a constant speed for a set duration.
   */
  public static Command driveForwardAuto(DriveSubsystem driveSubsystem) {
    return Commands.sequence(
        // Drive forward at 50% speed for 3 seconds
        Commands.run(() -> driveSubsystem.driveFieldOriented(
            new ChassisSpeeds(2.25, 0, 0)), driveSubsystem)
            .withTimeout(3),
        // Stop the robot
        Commands.runOnce(() -> driveSubsystem.stop(), driveSubsystem)
    ).withName("Drive Forward");
  }

  // Private constructor to prevent instantiation
  private Autos() {
    throw new UnsupportedOperationException("Utility class - do not instantiate");
  }
}
