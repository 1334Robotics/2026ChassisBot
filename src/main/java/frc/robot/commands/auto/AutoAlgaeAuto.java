package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Algae Removal Auto: Clear algae from field.
 */
public class AutoAlgaeAuto extends SequentialCommandGroup {
    
    public AutoAlgaeAuto(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> System.out.println("\n========== ALGAE REMOVAL AUTO ==========\n")),
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
