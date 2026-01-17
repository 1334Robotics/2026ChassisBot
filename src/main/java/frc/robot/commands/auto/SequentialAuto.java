package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Multi-Reef Reefscape Auto: Score on multiple reef branches.
 * 
 * Sequence:
 * 1. Start at Blue Alliance (1.5, 4.105)
 * 2. Drive to Coral Station pickup (1.2, 7.0) - pickup first piece
 * 3. Drive to Reef Left branch (7.57, 5.105)
 * 4. Score on left branch
 * 5. Return to safe zone
 * 
 * Total time: ~20 seconds
 * Demonstrates path planning around reef
 */
public class SequentialAuto extends SequentialCommandGroup {
    
    public SequentialAuto(DriveSubsystem drive) {
        addCommands(
            // Startup diagnostics
            Commands.runOnce(() -> {
                System.out.println("\n========== SEQUENTIAL AUTO (REEFSCAPE) ==========");
                Pose2d startPose = drive.getPose();
                System.out.println("Starting position: (" + 
                    String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
                System.out.println("Waypoints:");
                System.out.println("  1. Coral Station: (1.2, 7.0) - Pickup");
                System.out.println("  2. Reef Left: (7.57, 5.105) - Score");
                System.out.println("  3. Safe Zone: (2.5, 1.5) - Return");
                System.out.println("=================================================\n");
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
                2.0, 6.0
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
            })
        );
    }
}
