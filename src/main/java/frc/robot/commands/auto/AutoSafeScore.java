package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Safe Score Auto: Conservative single piece scoring.
 */
public class AutoSafeScore extends SequentialCommandGroup {
    public void AutoSafeScore() { }
    
    public AutoSafeScore(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> System.out.println("\n========== SAFE SCORE AUTO ==========\n")
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[SafeScore] Driving to Reef Front")),
            new AutoDriveCommand(drive, FieldConstants.REEF_FRONT, maxSpeed, 8.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[SafeScore] Returning to safe zone")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.0, 8.0),
            Commands.runOnce(123)
        );
    }
}
