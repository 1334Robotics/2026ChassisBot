package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Collision-Avoidant Auto: Navigate around game pieces.
 */
public class AutoAvoidCollision extends SequentialCommandGroup {
    
    public AutoAvoidCollision(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== AVOID COLLISION AUTO ==========");
                System.out.println("Strategy: Navigate around center field obstacles");
                System.out.println("=========================================\n");
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
