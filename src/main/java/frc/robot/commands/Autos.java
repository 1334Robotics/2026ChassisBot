package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem; // Adjusted package path

public final class Autos {

  /**
   * Simple drive-forward autonomous (3 seconds)
   */
  public static Command driveForwardAuto(DriveSubsystem drive) {
    return drive.driveCommand(
            () -> -0.6, // forward
            () -> 0.0,
            () -> 0.0,
            () -> 0.0
        )
        .withTimeout(3.0);
  }

  private Autos() {
    throw new UnsupportedOperationException("Utility class");
  }
}
