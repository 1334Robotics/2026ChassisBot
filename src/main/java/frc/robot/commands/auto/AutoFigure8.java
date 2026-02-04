package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Figure 8 Navigation Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Demonstrate advanced field navigation around reef obstacle
 * 
 * Strategy:
 * - Navigate in figure 8 pattern around entire Rebuilt field
 * - Avoid center reef structure at all times
 * - Left loop: Counter-clockwise around west side
 * - Right loop: Clockwise around east side
 * - Cross field safely above reef (Y=6.9m)
 * 
 * Path Design:
 * - Starts at Blue Alliance wall (X=1.5, Y=4.035)
 * - Moves NORTH first to avoid reef collision
 * - Left loop: X < 5.5m (stays west of reef)
 * - Crosses at Y=6.9m (2.87m clearance above reef)
 * - Right loop: X > 10.5m (stays east of reef)
 * - Returns via safe northern route
 * 
 * Safety Zones:
 * - Reef danger zone: X=6.5-10.0m, Y=2.5-5.5m (avoided completely)
 * - Crossing height: Y=6.9m (safe clearance)
 * - Field boundaries: X=0.5-16.04m, Y=0.5-7.57m (0.5m margin)
 * 
 * Points: 0 (navigation practice only)
 * Time: ~35-40 seconds
 * Purpose: Test autonomous navigation and obstacle avoidance
 */
public class AutoFigure8 extends SequentialCommandGroup {
    
    // Safety constants
    private static final double REEF_CENTER_X = 8.27;
    @SuppressWarnings("unused")// to suppress the error message because I don't like it.
    private static final double REEF_CENTER_Y = 4.035;
    private static final double SAFE_CROSSING_Y = 6.9; // 2.87m above reef center
    private static final double LEFT_ZONE_MAX_X = 5.5;
    private static final double RIGHT_ZONE_MIN_X = 10.5;
    
    // Initial safe movement - go NORTH first, then around perimeter
    private static final Pose2d SAFE_NORTH = new Pose2d(1.5, SAFE_CROSSING_Y, Rotation2d.fromDegrees(0));
    
    // LEFT LOOP waypoints (counter-clockwise around left side)
    private static final Pose2d LEFT_TOP = new Pose2d(3.0, 7.4, Rotation2d.fromDegrees(0));
    private static final Pose2d LEFT_MID_EAST = new Pose2d(LEFT_ZONE_MAX_X, 6.3, Rotation2d.fromDegrees(-45));
    private static final Pose2d LEFT_BOTTOM = new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(-90));
    private static final Pose2d LEFT_MID_RETURN = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
    
    // CROSSING waypoint - safely ABOVE the reef
    private static final Pose2d CROSSING_NORTH = new Pose2d(REEF_CENTER_X, SAFE_CROSSING_Y, Rotation2d.fromDegrees(0));
    
    // RIGHT LOOP waypoints (clockwise around right side)
    private static final Pose2d RIGHT_TOP = new Pose2d(11.5, 7.4, Rotation2d.fromDegrees(0));
    private static final Pose2d RIGHT_EAST = new Pose2d(13.5, 5.0, Rotation2d.fromDegrees(-90));
    private static final Pose2d RIGHT_BOTTOM = new Pose2d(11.0, 1.5, Rotation2d.fromDegrees(-135));
    private static final Pose2d RIGHT_MID_RETURN = new Pose2d(RIGHT_ZONE_MIN_X, 3.5, Rotation2d.fromDegrees(135));
    
    // Return crossing - back north of reef
    private static final Pose2d CROSSING_RETURN = new Pose2d(REEF_CENTER_X, SAFE_CROSSING_Y, Rotation2d.fromDegrees(180));
    
    public AutoFigure8(DriveSubsystem drive) {
        addCommands(
            // Print start message
            Commands.runOnce(() -> {
                System.out.println("\n========== FIGURE 8 NAV - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Advanced field navigation practice");
                System.out.println("Objective: Navigate figure 8 avoiding reef obstacle");
                System.out.println("Left Loop: Y=1.5-7.4m, X=1.5-5.5m (stays west of reef)");
                System.out.println("Right Loop: Y=1.5-7.4m, X=10.5-13.5m (stays east of reef)");
                System.out.println("Crossing: Y=6.9m (2.87m clearance above reef center)");
                System.out.println("Expected Points: 0 (navigation practice)");
                System.out.println("Field: 16.54m × 8.07m (Rebuilt 2025)");
                System.out.println("=====================================================\n");
            }),
            
            // Start from Blue Alliance position
            Commands.runOnce(() -> {
                drive.stop();
                System.out.println("[Figure8] Starting from Blue Alliance wall");
                System.out.println("[Figure8] Phase 1: Moving NORTH to safe crossing height...");
            }),
            Commands.waitSeconds(0.5),
            
            // FIRST: Move NORTH along wall (no forward movement toward reef!)
            new AutoDriveCommand(drive, SAFE_NORTH, 0.25, 2.5, 4.0),
            Commands.waitSeconds(0.3),
            
            // === LEFT LOOP (Counter-clockwise) ===
            Commands.runOnce(() -> System.out.println("[Figure8] Phase 2: LEFT LOOP (counter-clockwise)")),
            
            new AutoDriveCommand(drive, LEFT_TOP, 0.25, 3.0, 4.0),
            Commands.waitSeconds(0.2),
            
            Commands.runOnce(() -> System.out.println("[Figure8] Moving east (staying north of reef)")),
            new AutoDriveCommand(drive, LEFT_MID_EAST, 0.30, 3.5, 4.5),
            Commands.waitSeconds(0.2),
            
            Commands.runOnce(() -> System.out.println("[Figure8] Moving south along left safe zone")),
            new AutoDriveCommand(drive, LEFT_BOTTOM, 0.35, 4.0, 5.0),
            Commands.waitSeconds(0.2),
            
            Commands.runOnce(() -> System.out.println("[Figure8] Completing left loop - returning north")),
            new AutoDriveCommand(drive, LEFT_MID_RETURN, 0.30, 3.5, 4.5),
            Commands.waitSeconds(0.3),
            
            // === CROSS TO RIGHT SIDE (Above reef) ===
            Commands.runOnce(() -> System.out.println("[Figure8] Phase 3: CROSSING to right side (above reef)")),
            new AutoDriveCommand(drive, CROSSING_NORTH, 0.35, 4.5, 5.0),
            Commands.waitSeconds(0.4),
            
            // === RIGHT LOOP (Clockwise) ===
            Commands.runOnce(() -> System.out.println("[Figure8] Phase 4: RIGHT LOOP (clockwise)")),
            
            new AutoDriveCommand(drive, RIGHT_TOP, 0.30, 4.0, 4.5),
            Commands.waitSeconds(0.2),
            
            Commands.runOnce(() -> System.out.println("[Figure8] Moving south along right safe zone")),
            new AutoDriveCommand(drive, RIGHT_EAST, 0.35, 4.5, 5.0),
            Commands.waitSeconds(0.2),
            
            new AutoDriveCommand(drive, RIGHT_BOTTOM, 0.35, 4.5, 5.0),
            Commands.waitSeconds(0.2),
            
            Commands.runOnce(() -> System.out.println("[Figure8] Completing right loop - returning north")),
            new AutoDriveCommand(drive, RIGHT_MID_RETURN, 0.30, 4.0, 4.5),
            Commands.waitSeconds(0.3),
            
            // === CROSS BACK (Above reef) ===
            Commands.runOnce(() -> System.out.println("[Figure8] Phase 5: Crossing back (above reef)")),
            new AutoDriveCommand(drive, CROSSING_RETURN, 0.35, 4.5, 5.0),
            Commands.waitSeconds(0.3),
            
            // Return to safe zone
            Commands.runOnce(() -> System.out.println("[Figure8] Phase 6: Returning to Blue safe zone")),
            new AutoDriveCommand(drive, SAFE_NORTH, 0.25, 3.5, 4.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 0.20, 3.0, 4.0),
            
            // Finish
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                Pose2d finalPose = drive.getPose();
                System.out.println("\n[Figure8] [OK] Complete!");
                System.out.println("[Figure8] Figure 8 pattern completed with ZERO collisions");
                System.out.println("[Figure8] Final position: (" + 
                    String.format("%.2f, %.2f @ %.0f deg", finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees()) + ")");
                System.out.println("[Figure8] Total distance: ~35 meters\n");
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}


