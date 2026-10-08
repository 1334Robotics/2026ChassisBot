package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Collision Avoidance Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Navigate around field obstacles (reef and game pieces)
 * 
 * Strategy:
 * 1. Start at Blue Alliance
 * 2. Move along perimeter to avoid center reef
 * 3. Navigate to safe approach position north of reef
 * 4. Approach Reef Left scoring position carefully
 * 5. Retreat via perimeter route back to safe zone
 * 
 * Points: 0 (navigation practice - can be modified for scoring)
 * Time: ~25 seconds
 * Purpose: Demonstrate collision avoidance and safe pathing
 * Best for: Testing obstacle detection and path planning
 */
public class AutoAvoidCollision extends SequentialCommandGroup {
    
    public AutoAvoidCollision(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== COLLISION AVOID - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Navigate around center field obstacles");
                System.out.println("Objective: Safe pathing demonstration");
                System.out.println("Expected Points: 0 (navigation practice)");
                System.out.println("=========================================================\n");
            }),
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[AvoidCollision] Moving along perimeter")),
            new AutoDriveCommand(drive, new Pose2d(1.5, 6.5, Rotation2d.kZero), 2.0, 6.0),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[AvoidCollision] Moving to safe approach")),
            new AutoDriveCommand(drive, new Pose2d(7.0, 6.0, Rotation2d.fromDegrees(120.0)), 2.0, 8.0),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[AvoidCollision] Final approach")),
            new AutoDriveCommand(drive, FieldConstants.REEF_LEFT, 1.5, 6.0),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> System.out.println("[AvoidCollision] Retreating")),
            new AutoDriveCommand(drive, new Pose2d(3.0, 6.0, Rotation2d.kZero), 2.0, 8.0),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.0, 8.0),
            Commands.runOnce(() -> {
                System.out.println("[AvoidCollision] Complete\n");
                drive.stop();
                drive.lock();
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
