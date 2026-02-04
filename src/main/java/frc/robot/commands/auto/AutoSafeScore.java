package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Safe Score Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Conservative, reliable single coral scoring
 * 
 * Strategy:
 * 1. Start with preloaded coral
 * 2. Drive to Reef Front (nearest, safest approach)
 * 3. Score coral on Level 1 (3 points)
 * 4. Return to safe zone immediately
 * 
 * Points: 3 points (guaranteed)
 * Time: ~12-15 seconds
 * Best for: Reliability, untested robots, practice matches
 */
public class AutoSafeScore extends SequentialCommandGroup {
    
    public AutoSafeScore(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== SAFE SCORE - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Conservative single coral scoring");
                System.out.println("Expected Points: 3 (1 coral, guaranteed)");
                System.out.println("===================================================\n");
            }),
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[SafeScore] Driving to Reef Front")),
            new AutoDriveCommand(drive, FieldConstants.REEF_FRONT, 2.0, 8.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[SafeScore] Returning to safe zone")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.0, 8.0),
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[SafeScore] Complete\n");
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
