package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Processor Scoring Auto: Score pieces in processor.
 */
public class AutoProcessorAuto extends SequentialCommandGroup {
    
    public AutoProcessorAuto(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> System.out.println("\n========== PROCESSOR AUTO ==========\n")),
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[Processor] Driving to processor")),
            new AutoDriveCommand(drive, FieldConstants.PROCESSOR, 2.5, 8.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[Processor] Returning")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.5, 8.0),
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[Processor] Complete\n");
            })
        );
    }
}
