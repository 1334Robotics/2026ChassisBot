package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Score preloaded coral on reef center structure
 * 
 * Strategy:
 * 1. Start at Blue Alliance starting zone with preloaded coral
 * 2. Drive to Reef Front scoring position (nearest branch)
 * 3. Score coral on reef (Level 1 - 3 points)
 * 4. Return to safe zone for endgame positioning
 * 
 * Points: 3 points (1 coral on reef)
 * Time: ~12 seconds
 * Reliability: High - simple path, minimal risk
 */
public class SimpleAuto extends SequentialCommandGroup {
    
    public SimpleAuto(DriveSubsystem drive) {
        addCommands(
            // Startup diagnostics
            Commands.runOnce(() -> {
                System.out.println("\n========== SIMPLE AUTO - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Score preloaded coral on reef");
                Pose2d startPose = drive.getPose();
                System.out.println("Starting position: (" + 
                    String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
                System.out.println("Target: Reef Front (8.27, 2.5)");
                System.out.println("Expected Points: 3 (Level 1 coral)");
                System.out.println("Field: 16.54m × 8.07m (Rebuilt 2025)");
                System.out.println("====================================================\n");
            }),
            
            // Step 1: Confirm starting position and stabilize
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 1: Confirming at valid field position");
                drive.stop();
            }),
            Commands.waitSeconds(0.5),
            
            // Step 2: Drive to Reef Front approach
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 2: Driving to Reef front (8.77, 2.6)");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.REEF_FRONT,  // Approaches from front
                2.0,  // max speed (m/s)
                8.0   // timeout
            ),
            
            // Step 3: Simulate scoring
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 3: Attempting to score...");
                drive.stop();
            }),
            Commands.waitSeconds(1.5),  // Time for intake/mechanism
            
            // Step 4: Return to safe zone
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 4: Returning to safe zone");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.BLUE_SAFE_ZONE,
                2.5,
                8.0
            ),
            
            // Finish
            Commands.runOnce(() -> {
                Pose2d finalPose = drive.getPose();
                System.out.println("[SimpleAuto] Complete at (" + 
                    String.format("%.2f, %.2f", finalPose.getX(), finalPose.getY()) + ")");
                drive.stop();
                drive.lock();
                System.out.println();
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
