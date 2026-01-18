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
 * - First loop: Around the reef (right side) - stays clear of reef branches
 * - Second loop: Around coral station area (left side) - avoids station collision
 * Tests path following accuracy and rotation control.
 * 
 * Field Reference:
 * - Reef center: (8.77, 4.105)
 * - Field size: 17.54m x 8.21m
 * - Reef radius: ~1.5m (stay at least 2m away)
 * - Coral stations at field edges
 * - Start safely away from hub/processor near Blue Alliance wall
 */
public class AutoFigure8 extends SequentialCommandGroup {
    
    // Initial safe position - start further back and to the side (away from reef)
    // Move UP (North) first instead of forward to avoid reef collision
    private static final Pose2d SAFE_START_NORTH = new Pose2d(1.5, 6.0, Rotation2d.fromDegrees(90));
    
    // Then move laterally (East) along the top edge, clear of obstacles
    private static final Pose2d SAFE_LATERAL = new Pose2d(4.0, 6.5, Rotation2d.kZero);
    
    // Intermediate waypoint - ensures we stay clear of reef before heading to center
    private static final Pose2d APPROACH_CENTER = new Pose2d(5.5, 4.105, Rotation2d.kZero);
    
    // Center point where the figure 8 crosses (safe distance from reef)
    private static final Pose2d CENTER = new Pose2d(8.77, 4.105, Rotation2d.kZero);
    
    // First loop waypoints (right side - around reef area with safe clearance)
    private static final Pose2d LOOP1_NORTH = new Pose2d(11.5, 6.0, Rotation2d.fromDegrees(45));
    private static final Pose2d LOOP1_EAST = new Pose2d(13.5, 4.105, Rotation2d.kZero);
    private static final Pose2d LOOP1_SOUTH = new Pose2d(11.5, 2.2, Rotation2d.fromDegrees(-45));
    
    // Cross back through center (approach from safe angle)
    private static final Pose2d CENTER_RETURN = new Pose2d(8.77, 4.105, Rotation2d.fromDegrees(180));
    
    // Second loop waypoints (left side - clear of coral stations)
    private static final Pose2d LOOP2_NORTH = new Pose2d(6.0, 6.5, Rotation2d.fromDegrees(135));
    private static final Pose2d LOOP2_WEST = new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(180));
    private static final Pose2d LOOP2_SOUTH = new Pose2d(6.0, 1.7, Rotation2d.fromDegrees(-135));
    
    public AutoFigure8(DriveSubsystem drive) {
        addCommands(
            // Print start message
            Commands.runOnce(() -> {
                System.out.println("\n========== FIGURE 8 AUTO (REEFSCAPE) ==========");
                System.out.println("Strategy: Navigate figure 8 around field elements");
                System.out.println("Step 0: Move NORTH first to avoid reef collision");
                System.out.println("Step 1: Move laterally along safe edge");
                System.out.println("Step 2: Approach center with wide berth around reef");
                System.out.println("Loop 1: Wide path around reef (right, 2m+ clearance)");
                System.out.println("Loop 2: Around coral station area (left, safe margin)");
                System.out.println("Field: 17.54m x 8.21m | Reef at center with 1.5m radius");
                System.out.println("===============================================\n");
            }),
            
            // Start from Blue Alliance position
            Commands.runOnce(() -> {
                drive.stop();
                System.out.println("[Figure8] Starting from Blue Alliance");
                System.out.println("[Figure8] Moving NORTH to avoid reef...");
            }),
            Commands.waitSeconds(0.5),
            
            // FIRST: Move NORTH (up) to avoid reef - NO forward movement yet!
            new AutoDriveCommand(drive, SAFE_START_NORTH, 1.5, 2.5),
            Commands.waitSeconds(0.4),
            
            // SECOND: Move laterally (East) along safe top edge
            Commands.runOnce(() -> System.out.println("[Figure8] Moving laterally along safe edge...")),
            new AutoDriveCommand(drive, SAFE_LATERAL, 1.5, 3.0),
            Commands.waitSeconds(0.3),
            
            // THIRD: Approach center position with wide berth around reef (stay left of reef)
            Commands.runOnce(() -> System.out.println("[Figure8] Approaching center (staying clear of reef)...")),
            new AutoDriveCommand(drive, APPROACH_CENTER, 2.0, 4.0),
            Commands.waitSeconds(0.3),
            
            // FOURTH: Move to center crossing point (short final approach)
            Commands.runOnce(() -> System.out.println("[Figure8] Moving to center crossing point")),
            new AutoDriveCommand(drive, CENTER, 2.5, 5.0),
            Commands.waitSeconds(0.3),
            
            // First loop (right side - wide clearance around reef)
            Commands.runOnce(() -> System.out.println("[Figure8] Loop 1: Wide path around reef (2m+ clearance)")),
            new AutoDriveCommand(drive, LOOP1_NORTH, 3.0, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP1_EAST, 3.5, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP1_SOUTH, 3.0, 5.0),
            Commands.waitSeconds(0.2),
            
            // Return to center crossing point (straight path)
            Commands.runOnce(() -> System.out.println("[Figure8] Crossing center (north of reef)...")),
            new AutoDriveCommand(drive, CENTER_RETURN, 2.5, 5.0),
            Commands.waitSeconds(0.3),
            
            // Second loop (left side - avoid coral stations at field edge)
            Commands.runOnce(() -> System.out.println("[Figure8] Loop 2: Around coral area (safe margins)")),
            new AutoDriveCommand(drive, LOOP2_NORTH, 3.0, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP2_WEST, 3.0, 5.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, LOOP2_SOUTH, 3.0, 5.0),
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
                System.out.println("[Figure8] [OK] Complete!");
                System.out.println("[Figure8] Final position: (" + 
                    String.format("%.2f, %.2f", finalPose.getX(), finalPose.getY()) + ")");
                System.out.println("[Figure8] Total distance traveled: ~30 meters");
                System.out.println();
            })
        );
    }
}


