package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Processor Scoring Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Score coral in processor station
 * 
 * Strategy:
 * 1. Start with preloaded coral
 * 2. Navigate to Processor station
 * 3. Score coral in processor (4 points - higher value!)
 * 4. Return to safe zone
 * 
 * Points: 4 points (processor scoring bonus)
 * Time: ~15 seconds
 * Note: Processor scores are worth more than reef (4 pts vs 3 pts)
 */
public class AutoProcessorAuto extends SequentialCommandGroup {
    
    public AutoProcessorAuto(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== PROCESSOR AUTO - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Score coral in processor station");
                System.out.println("Expected Points: 4 (processor bonus)");
                System.out.println("=======================================================\n");
            }),
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
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
