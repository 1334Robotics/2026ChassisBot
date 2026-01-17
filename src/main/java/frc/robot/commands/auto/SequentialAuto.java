package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Multi-waypoint autonomous routine with return home.
 * 
 * Sequence:
 * 1. Reset to origin (0, 0)
 * 2. Wait for stabilization
 * 3. Drive to (2.0, 0.0) facing 0°
 * 4. Drive to (2.0, 2.0) facing 90°
 * 5. Return to origin (0.0, 0.0) facing 0°
 * 6. Lock wheels
 * 
 * Total time: ~16 seconds
 * Useful for testing multi-point path following and odometry accuracy
 */
public class SequentialAuto extends SequentialCommandGroup {
    
    public SequentialAuto(DriveSubsystem drive) {
        addCommands(
            // Initialize: Reset to known position
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Initializing - resetting to origin (0, 0)");
                drive.stop();
                drive.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.kZero));
            }),
            
            // Wait for stabilization
            Commands.waitSeconds(0.5),
            
            // Waypoint 1: Forward 2 meters
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Waypoint 1: Driving to (2.0, 0.0)");
            }),
            new AutoDriveCommand(drive, new Pose2d(2.0, 0.0, Rotation2d.kZero), 2.0, 5.0),
            
            Commands.waitSeconds(0.5),
            
            // Waypoint 2: Strafe left and rotate 90 degrees
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Waypoint 2: Driving to (2.0, 2.0) at 90°");
            }),
            new AutoDriveCommand(drive, new Pose2d(2.0, 2.0, new Rotation2d(Math.PI / 2)), 2.0, 5.0),
            
            Commands.waitSeconds(0.5),
            
            // Return home
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Returning to origin (0.0, 0.0)");
            }),
            new AutoDriveCommand(drive, new Pose2d(0.0, 0.0, Rotation2d.kZero), 2.0, 5.0),
            
            // Finish: Stop and lock
            Commands.runOnce(() -> {
                System.out.println("[SequentialAuto] Complete - locking wheels");
                drive.stop();
                drive.lock();
            })
        );
    }
}
