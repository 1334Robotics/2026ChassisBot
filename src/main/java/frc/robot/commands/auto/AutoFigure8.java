package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Figure 8 Auto: Drives in a figure 8 pattern around the Reefscape field.
 * Creates two loops that navigate around key field elements:
 * - First loop: Around the reef (right side)
 * - Second loop: Around coral station area (left side)
 * Tests path following accuracy and rotation control.
 */
public class AutoFigure8 extends SequentialCommandGroup {
    
    // Center point where the figure 8 crosses (near reef center)
    private static final Pose2d CENTER = new Pose2d(8.77, 4.105, Rotation2d.kZero);
    
    // First loop waypoints (right side - around reef area)
    private static final Pose2d LOOP1_NORTH = new Pose2d(10.5, 5.5, Rotation2d.fromDegrees(45));
    private static final Pose2d LOOP1_EAST = new Pose2d(12.0, 4.105, Rotation2d.kZero);
    private static final Pose2d LOOP1_SOUTH = new Pose2d(10.5, 2.5, Rotation2d.fromDegrees(-45));
    
    // Cross back through center
    private static final Pose2d CENTER_RETURN = new Pose2d(8.77, 4.105, Rotation2d.fromDegrees(180));
    
    // Second loop waypoints (left side - around coral station area)
    private static final Pose2d LOOP2_NORTH = new Pose2d(5.5, 6.0, Rotation2d.fromDegrees(135));
    private static final Pose2d LOOP2_WEST = new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(180));
    private static final Pose2d LOOP2_SOUTH = new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(-135));
    
    public AutoFigure8(DriveSubsystem drive) {
        addCommands(
            // Print start message
            Commands.runOnce(() -> {
                System.out.println("\n========== FIGURE 8 AUTO (REEFSCAPE) ==========");
                System.out.println("Strategy: Navigate figure 8 around field elements");
                System.out.println("Loop 1: Around reef area (right side)");
                System.out.println("Loop 2: Around coral station area (left side)");
                System.out.println("Field: 17.54m x 8.21m");
                System.out.println("===============================================\n");
            }),
            
            // Start from Blue Alliance position
            Commands.runOnce(() -> {
                drive.stop();
                System.out.println("[Figure8] Starting from Blue Alliance");
                System.out.println("[Figure8] Moving to center crossing point");
            }),
            Commands.waitSeconds(0.5),
            
            // Move to center
            new AutoDriveCommand(drive, CENTER, 2.5, 6.0),
            Commands.waitSeconds(0.3),
            
            // First loop (right side - clockwise around reef)
            Commands.runOnce(() -> System.out.println("[Figure8] Loop 1: Around reef area (clockwise)")),
            new AutoDriveCommand(drive, LOOP1_NORTH, 2.5, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP1_EAST, 2.5, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP1_SOUTH, 2.5, 5.0),
            Commands.waitSeconds(0.2),
            
            // Return to center crossing point
            Commands.runOnce(() -> System.out.println("[Figure8] Crossing center...")),
            new AutoDriveCommand(drive, CENTER_RETURN, 2.5, 5.0),
            Commands.waitSeconds(0.3),
            
            // Second loop (left side - counter-clockwise around coral station)
            Commands.runOnce(() -> System.out.println("[Figure8] Loop 2: Around coral station (counter-clockwise)")),
            new AutoDriveCommand(drive, LOOP2_NORTH, 2.5, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP2_WEST, 2.5, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP2_SOUTH, 2.5, 5.0),
            Commands.waitSeconds(0.2),
            
            // Return to center and then safe zone
            Commands.runOnce(() -> System.out.println("[Figure8] Returning to safe zone")),
            new AutoDriveCommand(drive, CENTER, 2.5, 5.0),
            Commands.waitSeconds(0.3),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.5, 6.0),
            
            // Finish
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                Pose2d finalPose = drive.getPose();
                System.out.println("[Figure8] ✓ Complete!");
                System.out.println("[Figure8] Final position: (" + 
                    String.format("%.2f, %.2f", finalPose.getX(), finalPose.getY()) + ")");
                System.out.println("[Figure8] Total distance traveled: ~30 meters");
                System.out.println();
            })
        );
    }
}


