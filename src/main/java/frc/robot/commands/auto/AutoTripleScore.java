package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Triple Score Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Score on all three reef branches (no pickup cycles)
 * 
 * Strategy:
 * Assumes robot has 3 preloaded corals (or uses ground corals)
 * 1. Score on Reef Front branch (0°)
 * 2. Score on Reef Left branch (120°)
 * 3. Score on Reef Right branch (60°)
 * 4. Return to safe zone
 * 
 * Points: 9 points (3 corals × 3 points each)
 * Time: ~20 seconds
 * Note: Requires multi-coral storage or ground pickup capability
 */
public class AutoTripleScore extends SequentialCommandGroup {
    
    public AutoTripleScore(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== TRIPLE SCORE - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Score on all 3 reef branches");
                System.out.println("Expected Points: 9 (3 corals × 3 pts)");
                System.out.println("======================================================\n");
            }),
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
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
