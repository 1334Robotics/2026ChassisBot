package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Complex Path Reefscape Auto: Complete reef scoring pattern.
 * 
 * Strategy:
 * 1. Start at Blue Alliance (preloaded coral)
 * 2. Score preload on Reef Front (nearest)
 * 3. Pickup coral at Coral Station
 * 4. Score on Reef Left (120°)
 * 5. Pickup coral at Coral Station
 * 6. Score on Reef Right (60°)
 * 7. Return to safe zone
 * 
 * Total: 3 pieces scored on reef
 * Estimated time: ~25-30 seconds
 */
public class AutoComplexPath extends SequentialCommandGroup {
    
    public AutoComplexPath(DriveSubsystem drive) {
        addCommands(
            // Startup diagnostics
            Commands.runOnce(() -> {
                System.out.println("\n========== COMPLEX PATH AUTO (REEFSCAPE) ==========");
                Pose2d startPose = drive.getPose();
                System.out.println("Starting position: (" + 
                    String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
                System.out.println("\nScoring Sequence:");
                System.out.println("  1. Reef Front (preload): (8.77, 2.6) @ 0°");
                System.out.println("  2. Pickup: Coral Station (1.2, 7.0)");
                System.out.println("  3. Reef Left: (7.57, 5.105) @ 120°");
                System.out.println("  4. Pickup: Coral Station (1.2, 7.0)");
                System.out.println("  5. Reef Right: (10.0, 5.105) @ 60°");
                System.out.println("  6. Safe Zone: (2.5, 1.5) @ 0°");
                System.out.println("===================================================\n");
            }),
            
            // Initialize
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Ready with preloaded coral");
                drive.stop();
            }),
            Commands.waitSeconds(0.5),
            
            // Score 1: Reef Front (preload)
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Score 1/3: Reef Front (preload)");
            }),
            new AutoDriveCommand(drive, FieldConstants.REEF_FRONT, 2.5, 8.0),
            Commands.waitSeconds(1.0), // Score time
            
            // Pickup 1: Coral Station
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Pickup 1/2: Coral Station");
            }),
            new AutoDriveCommand(drive, FieldConstants.BLUE_CORAL_STATION, 2.5, 8.0),
            Commands.waitSeconds(1.0), // Pickup time
            
            // Score 2: Reef Left
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Score 2/3: Reef Left (120°)");
            }),
            new AutoDriveCommand(drive, FieldConstants.REEF_LEFT, 2.5, 8.0),
            Commands.waitSeconds(1.0), // Score time
            
            // Pickup 2: Coral Station
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Pickup 2/2: Coral Station");
            }),
            new AutoDriveCommand(drive, FieldConstants.BLUE_CORAL_STATION, 2.5, 8.0),
            Commands.waitSeconds(1.0), // Pickup time
            
            // Score 3: Reef Right
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Score 3/3: Reef Right (60°)");
            }),
            new AutoDriveCommand(drive, FieldConstants.REEF_RIGHT, 2.5, 8.0),
            Commands.waitSeconds(1.0), // Score time
            
            // Return to safe zone
            Commands.runOnce(() -> {
                System.out.println("[ComplexPath] Returning to safe zone");
            }),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.5, 10.0),
            
            // Finish
            Commands.runOnce(() -> {
                Pose2d finalPose = drive.getPose();
                System.out.println("[ComplexPath] [OK] Complete at (" + 
                    String.format("%.2f, %.2f", finalPose.getX(), finalPose.getY()) + ")");
                System.out.println("[ComplexPath] [OK] Total pieces scored: 3");
                System.out.println("[ComplexPath] [OK] Reef branches covered: Front, Left, Right");
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
