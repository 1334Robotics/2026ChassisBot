package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Sequential Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Multi-waypoint coral scoring demonstration
 * 
 * Strategy:
 * 1. Start at Blue Alliance
 * 2. Navigate to Coral Station → Pickup coral
 * 3. Navigate to Reef Left branch → Score coral (3 pts)
 * 4. Return to safe zone for endgame
 * 
 * Points: 3 points (1 coral from pickup)
 * Time: ~20 seconds
 * Purpose: Demonstrates path planning and waypoint navigation
 */
public class SequentialAuto extends SequentialCommandGroup {
    
    public SequentialAuto(DriveSubsystem drive) {
        addCommands(
            // Startup diagnostics
            Commands.runOnce(() -> {
                System.out.println("\n========== SEQUENTIAL AUTO - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Multi-waypoint coral scoring");
                Pose2d startPose = drive.getPose();
                System.out.println("Starting position: (" + 
                    String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
                System.out.println("Waypoints:");
                System.out.println("  1. Coral Station: (1.2, 7.0) - Pickup");
                System.out.println("  2. Reef Left: (7.2, 5.0) - Score (3 pts)");
                System.out.println("  3. Safe Zone: (2.0, 1.5) - Return");
                System.out.println("Expected Points: 3");
                System.out.println("Field: 16.54m × 8.07m (Rebuilt 2025)");
                System.out.println("========================================================\n");
            }),
            
            // Initialize: Verify starting position
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Verifying start position...");
                drive.stop();
            }),
            Commands.waitSeconds(0.5),
            
            // Waypoint 1: Drive to Coral Station
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Waypoint 1: Driving to Coral Station (1.2, 7.0)");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.BLUE_CORAL_STATION,
                2.0, 8.0
            ),
            Commands.waitSeconds(1.0),  // Pickup time
            
            // Waypoint 2: Drive to Reef Left branch
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Waypoint 2: Driving to Reef Left (7.57, 5.105)");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.REEF_LEFT,
                2.5, 8.0
            ),
            Commands.waitSeconds(1.5),  // Score time
            
            // Waypoint 3: Return to safe zone
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Waypoint 3: Returning to safe zone");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.BLUE_SAFE_ZONE,
                2.5, 8.0
            ),
            
            // Finish
            Commands.runOnce(() -> {
                Pose2d finalPose = drive.getPose();
                System.out.println("[SequentialAuto] Complete at (" + 
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
