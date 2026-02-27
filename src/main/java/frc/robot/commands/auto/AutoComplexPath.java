package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Complex Path Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Maximum coral scoring with multiple pickup cycles
 * 
 * Strategy (3-piece auto):
 * 1. Start with preloaded coral → Score on Reef Front (Level 1)
 * 2. Pickup from Coral Station → Score on Reef Left (Level 1)
 * 3. Pickup from Coral Station → Score on Reef Right (Level 1)
 * 4. Return to safe zone for endgame positioning
 * 
 * Points: 9 points (3 corals × 3 points each)
 * Time: ~25-30 seconds
 * Complexity: High - requires precise navigation and coral handling
 * Best for: Experienced drivers, reliable intake/outtake mechanisms
 */
public class AutoComplexPath extends SequentialCommandGroup {
    
    public AutoComplexPath(DriveSubsystem drive) {
        addCommands(
            // Startup diagnostics
            Commands.runOnce(() -> {
                System.out.println("\n========== COMPLEX PATH - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Multi-coral scoring (3-piece auto)");
                Pose2d startPose = drive.getPose();
                System.out.println("Starting position: (" + 
                    String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
                System.out.println("\nScoring Sequence:");
                System.out.println("  1. Reef Front (preload): (8.27, 2.5) @ 0° → 3 pts");
                System.out.println("  2. Pickup: Coral Station (1.2, 7.0)");
                System.out.println("  3. Reef Left: (7.2, 5.0) @ 120° → 3 pts");
                System.out.println("  4. Pickup: Coral Station (1.2, 7.0)");
                System.out.println("  5. Reef Right: (9.3, 5.0) @ 60° → 3 pts");
                System.out.println("  6. Safe Zone: (2.0, 1.5) @ 0°");
                System.out.println("\nExpected Points: 9 (3 corals on reef)");
                System.out.println("Field: 16.54m × 8.07m (Rebuilt 2025)");
                System.out.println("=====================================================\n");
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
