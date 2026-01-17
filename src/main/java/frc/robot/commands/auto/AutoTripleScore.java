package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Triple Score Auto: Score on three different reef branches.
 */
public class AutoTripleScore extends SequentialCommandGroup {
    
    public AutoTripleScore(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> System.out.println("\n========== TRIPLE SCORE AUTO ==========\n")),
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[TripleScore] Score 1: Reef Front")),
            new AutoDriveCommand(drive, FieldConstants.REEF_FRONT, 2.5, 8.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[TripleScore] Score 2: Reef Left")),
            new AutoDriveCommand(drive, FieldConstants.REEF_LEFT, 2.5, 8.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[TripleScore] Score 3: Reef Right")),
            new AutoDriveCommand(drive, FieldConstants.REEF_RIGHT, 2.5, 8.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[TripleScore] Returning to safe zone")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.5, 10.0),
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[TripleScore] Complete - 3 pieces scored\n");
            })
        );
    }
}
