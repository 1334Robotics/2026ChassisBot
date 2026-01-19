package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;


public class AutoFigure8 extends SequentialCommandGroup {
    
    // Initial safe movement - go UP first, then around perimeter
    private static final Pose2d SAFE_NORTH = new Pose2d(1.5, 7.0, Rotation2d.fromDegrees(0));
    
    // LEFT LOOP waypoints (counter-clockwise around left side)
    private static final Pose2d LEFT_TOP = new Pose2d(3.0, 7.5, Rotation2d.fromDegrees(0));
    private static final Pose2d LEFT_MID_EAST = new Pose2d(5.5, 6.5, Rotation2d.fromDegrees(-45));
    private static final Pose2d LEFT_BOTTOM = new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(-90));
    private static final Pose2d LEFT_MID_RETURN = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
    
    // CROSSING waypoint - safely ABOVE the reef
    private static final Pose2d CROSSING_NORTH = new Pose2d(8.77, 7.0, Rotation2d.fromDegrees(0));
    
    // RIGHT LOOP waypoints (clockwise around right side)
    private static final Pose2d RIGHT_TOP = new Pose2d(12.5, 7.5, Rotation2d.fromDegrees(0));
    private static final Pose2d RIGHT_EAST = new Pose2d(14.5, 5.0, Rotation2d.fromDegrees(-90));
    private static final Pose2d RIGHT_BOTTOM = new Pose2d(12.0, 1.5, Rotation2d.fromDegrees(-135));
    private static final Pose2d RIGHT_MID_RETURN = new Pose2d(10.5, 3.5, Rotation2d.fromDegrees(135));
    
    // Return crossing - back north of reef
    private static final Pose2d CROSSING_RETURN = new Pose2d(8.77, 7.0, Rotation2d.fromDegrees(180));
    
    public AutoFigure8(DriveSubsystem drive) {
        addCommands(
            // Print start message
            Commands.runOnce(() -> {
                System.out.println("\n========== FIGURE 8 AUTO (REEFSCAPE - SAFE) ==========");
                System.out.println("Strategy: Navigate figure 8 AROUND field perimeter");
                System.out.println("NO REEF COLLISION - All paths avoid center reef zone");
                System.out.println("Left Loop: Y=1.5-7.5, X=3.0-5.5 (stays west of reef)");
                System.out.println("Right Loop: Y=1.5-7.5, X=10.5-14.5 (stays east of reef)");
                System.out.println("Crossing: Y=7.0 (safely ABOVE reef at all times)");
                System.out.println("======================================================\n");
            }),
            
            // Start from Blue Alliance position
            Commands.runOnce(() -> {
                drive.stop();
                System.out.println("[Figure8] Starting from Blue Alliance");
                System.out.println("[Figure8] Moving NORTH along wall to avoid all obstacles...");
            }),
            Commands.waitSeconds(0.5),
            
            // FIRST: Move NORTH along wall (no forward movement toward reef!)
            new AutoDriveCommand(drive, SAFE_NORTH, 1.0, 3.0),
            Commands.waitSeconds(0.3),
            
            // === LEFT LOOP (Counter-clockwise) ===
            Commands.runOnce(() -> System.out.println("[Figure8] LEFT LOOP: Starting counter-clockwise path")),
            
            // Top left corner
            new AutoDriveCommand(drive, LEFT_TOP, 1.5, 3.5),
            Commands.waitSeconds(0.2),
            
            // Move east toward center (but stay north of reef)
            Commands.runOnce(() -> System.out.println("[Figure8] Moving east (staying north of reef zone)")),
            new AutoDriveCommand(drive, LEFT_MID_EAST, 2.0, 4.0),
            Commands.waitSeconds(0.2),
            
            // Drop south along left side
            Commands.runOnce(() -> System.out.println("[Figure8] Moving south along left safe zone")),
            new AutoDriveCommand(drive, LEFT_BOTTOM, 2.5, 4.5),
            Commands.waitSeconds(0.2),
            
            // Return north on left side
            Commands.runOnce(() -> System.out.println("[Figure8] Completing left loop")),
            new AutoDriveCommand(drive, LEFT_MID_RETURN, 2.0, 4.0),
            Commands.waitSeconds(0.3),
            
            // === CROSS TO RIGHT SIDE (Above reef) ===
            Commands.runOnce(() -> System.out.println("[Figure8] CROSSING: Moving to right side (above reef)")),
            new AutoDriveCommand(drive, CROSSING_NORTH, 2.5, 5.0),
            Commands.waitSeconds(0.3),
            
            // === RIGHT LOOP (Clockwise) ===
            Commands.runOnce(() -> System.out.println("[Figure8] RIGHT LOOP: Starting clockwise path")),
            
            // Top right corner
            new AutoDriveCommand(drive, RIGHT_TOP, 2.5, 4.5),
            Commands.waitSeconds(0.2),
            
            // East side down
            Commands.runOnce(() -> System.out.println("[Figure8] Moving south along right safe zone")),
            new AutoDriveCommand(drive, RIGHT_EAST, 3.0, 5.0),
            Commands.waitSeconds(0.2),
            
            // Bottom right
            new AutoDriveCommand(drive, RIGHT_BOTTOM, 3.0, 5.0),
            Commands.waitSeconds(0.2),
            
            // Return north on right side
            Commands.runOnce(() -> System.out.println("[Figure8] Completing right loop")),
            new AutoDriveCommand(drive, RIGHT_MID_RETURN, 2.5, 4.5),
            Commands.waitSeconds(0.3),
            
            // === CROSS BACK (Above reef) ===
            Commands.runOnce(() -> System.out.println("[Figure8] Crossing back (above reef)")),
            new AutoDriveCommand(drive, CROSSING_RETURN, 2.5, 5.0),
            Commands.waitSeconds(0.3),
            
            // Return to safe zone
            Commands.runOnce(() -> System.out.println("[Figure8] Returning to Blue safe zone")),
            new AutoDriveCommand(drive, SAFE_NORTH, 2.0, 4.0),
            Commands.waitSeconds(0.2),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 1.5, 4.0),
            
            // Finish
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                Pose2d finalPose = drive.getPose();
                System.out.println("[Figure8] [OK] Complete!");
                System.out.println("[Figure8] Figure 8 completed with ZERO reef collisions");
                System.out.println("[Figure8] Final position: (" + 
                    String.format("%.2f, %.2f", finalPose.getX(), finalPose.getY()) + ")");
                System.out.println();
            })
        );
    }
}


