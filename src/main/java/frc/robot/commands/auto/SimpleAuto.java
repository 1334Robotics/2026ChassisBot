package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple Reefscape Auto: Drive to reef and score.
 * 
 * Sequence:
 * 1. Start at Blue Alliance (1.5, 4.105)
 * 2. Drive to Reef Front approach (8.77, 2.6)
 * 3. Simulate scoring (wait for mechanism)
 * 4. Return to safe zone
 * 
 * Total time: ~12 seconds
 */
public class SimpleAuto extends SequentialCommandGroup {
    
    public SimpleAuto(DriveSubsystem drive) {
        addCommands(
            // Startup diagnostics
            Commands.runOnce(() -> {
                System.out.println("\n========== SIMPLE AUTO (REEFSCAPE) ==========");
                Pose2d startPose = drive.getPose();
                System.out.println("Robot starting position: (" + 
                    String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
                System.out.println("Target: Reef Front approach (8.77, 2.6)");
                System.out.println("Field: 17.54m × 8.21m (Reefscape 2025)");
                System.out.println("==========================================\n");
            }),
            
            // Step 1: Confirm starting position and stabilize
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 1: Confirming at valid field position");
                drive.stop();
            }),
            Commands.waitSeconds(0.5),
            
            // Step 2: Drive to Reef Front approach
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 2: Driving to Reef front (8.77, 2.6)");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.REEF_FRONT,  // Approaches from front
                2.0,  // max speed (m/s)
                8.0   // timeout
            ),
            
            // Step 3: Simulate scoring
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 3: Attempting to score...");
                drive.stop();
            }),
            Commands.waitSeconds(1.5),  // Time for intake/mechanism
            
            // Step 4: Return to safe zone
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] Step 4: Returning to safe zone");
            }),
            new AutoDriveCommand(
                drive,
                FieldConstants.BLUE_SAFE_ZONE,
                2.5,
                8.0
            ),
            
            // Finish
            Commands.runOnce(() -> {
                Pose2d finalPose = drive.getPose();
                System.out.println("[SimpleAuto] Complete at (" + 
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
