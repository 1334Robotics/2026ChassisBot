package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Algae Removal Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Clear algae from designated field zones
 * 
 * Strategy:
 * 1. Start at Blue Alliance
 * 2. Navigate to left algae zone (X=4.0m)
 * 3. Clear algae (wait for mechanism)
 * 4. Navigate to right algae zone (X=13.0m)
 * 5. Clear algae (wait for mechanism)
 * 6. Hold position
 * 
 * Points: Variable (depends on algae cleared)
 * Time: ~18 seconds
 * Note: Algae removal provides bonus points and field control
 */
public class AutoAlgaeAuto extends SequentialCommandGroup {
    
    public AutoAlgaeAuto(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== ALGAE REMOVAL - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Clear algae from field zones");
                System.out.println("Target: 2 algae zones");
                System.out.println("Expected Points: Variable (bonus points)");
                System.out.println("======================================================\n");
            }),
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> System.out.println("[Algae] Patrolling algae zones")),
            new AutoDriveCommand(drive, new Pose2d(4.0, 4.0, Rotation2d.kZero), 2.0, 6.0),
            Commands.waitSeconds(0.5),
            new AutoDriveCommand(drive, new Pose2d(13.0, 4.0, Rotation2d.kZero), 2.0, 6.0),
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[Algae] Complete\n");
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
