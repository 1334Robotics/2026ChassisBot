package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Balance/Center Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Navigate to field center and maintain position
 * 
 * Strategy:
 * 1. Start at Blue Alliance
 * 2. Navigate to field center (8.27, 4.035)
 * 3. Hold position for balance/stability test
 * 4. Lock wheels to maintain position
 * 
 * Points: 0 (positioning practice)
 * Time: ~15 seconds
 * Purpose: Test center field navigation and position holding
 * Note: Can be modified for endgame balance/park scoring
 */
public class AutoBalance extends SequentialCommandGroup {
    
    private static final double BALANCE_POSITION_X = 8.27;
    private static final Pose2d BALANCE_POSITION = new Pose2d(BALANCE_POSITION_X, 4.035, Rotation2d.kZero);
    
    public AutoBalance(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== BALANCE/CENTER - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Navigate to center and hold position");
                System.out.println("Target: Field center (" + String.format("%.2f, %.2f", 
                    BALANCE_POSITION.getX(), BALANCE_POSITION.getY()) + ")");
                System.out.println("Expected Points: 0 (positioning practice)");
                System.out.println("========================================================\n");
            }),
            
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            
            Commands.runOnce(() -> System.out.println("[Balance] Moving to balance position")),
            new AutoDriveCommand(drive, BALANCE_POSITION, 2.0, 8.0),
            
            Commands.waitSeconds(0.5),
            
            Commands.runOnce(() -> System.out.println("[Balance] Maintaining position")),
            Commands.waitSeconds(2.0),
            
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[Balance] Complete - holding position\n");
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
