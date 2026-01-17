package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple autonomous routine: Drive forward 2 meters and stop.
 * 
 * Sequence:
 * 1. Reset odometry to known position (0, 0)
 * 2. Wait for systems to stabilize
 * 3. Use AutoDriveCommand to drive to (2.0, 0.0)
 * 4. Stop and lock
 * 
 * Total time: ~5 seconds
 */
public class SimpleAuto extends SequentialCommandGroup {
    
    public SimpleAuto(DriveSubsystem drive) {
        addCommands(
            // Step 1: Ensure we're at origin and stopped
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 1: Resetting odometry to origin (0, 0)");
                drive.stop();
                drive.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.kZero));
            }),
            
            // Step 2: Wait for modules to settle
            Commands.waitSeconds(0.5),
            
            // Step 3: Drive forward 2 meters
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 2: Starting drive to (2.0, 0.0)");
            }),
            
            new AutoDriveCommand(
                drive,
                new Pose2d(2.0, 0.0, Rotation2d.kZero), // Target: 2m forward, 0° rotation
                1.5,  // max speed (m/s)
                4.0   // timeout (seconds)
            ),
            
            // Step 4: Stop and secure
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 3: Stopping and locking wheels");
                drive.stop();
                drive.lock();
            })
        );
    }
}
