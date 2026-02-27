package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Pickup Cycles Auto - FIRST Rebuilt 2025 Strategy
 * 
 * Game: FIRST Reefscape Rebuilt
 * Objective: Practice rapid coral pickup and scoring cycles
 * 
 * Strategy (3-cycle auto):
 * 1. Score preloaded coral on Reef Front (3 pts)
 * 2. Cycle 1: Pickup from Coral Station → Score on Reef Left (3 pts)
 * 3. Cycle 2: Pickup from Coral Station → Score on Reef Right (3 pts)
 * 4. Return to safe zone
 * 
 * Points: 9 points (3 corals × 3 points each)
 * Time: ~22-25 seconds
 * Focus: Speed and efficiency in coral handling
 */
public class AutoPickupAuto extends SequentialCommandGroup {
    
    public AutoPickupAuto(DriveSubsystem drive) {
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== PICKUP CYCLES - FIRST REBUILT 2025 ==========");
                System.out.println("Strategy: Rapid coral pickup and scoring cycles");
                System.out.println("Target: 3 corals (preload + 2 pickups)");
                System.out.println("Expected Points: 9 (3 × 3 pts)");
                System.out.println("======================================================\n");
            }),
            
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.3),
            
            // Cycle 1: Preload to Reef Front
            Commands.runOnce(() -> System.out.println("[PickupCycles] Cycle 1: Scoring preload")),
            new AutoDriveCommand(drive, FieldConstants.REEF_FRONT, 2.5, 6.0),
            Commands.waitSeconds(0.8),
            
            // Pickup from Coral Station
            Commands.runOnce(() -> System.out.println("[PickupCycles] Cycle 1: Picking up coral")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_CORAL_STATION, 2.5, 6.0),
            Commands.waitSeconds(0.8),
            
            // Cycle 2: Score on Reef Left
            Commands.runOnce(() -> System.out.println("[PickupCycles] Cycle 2: Scoring on Reef Left")),
            new AutoDriveCommand(drive, FieldConstants.REEF_LEFT, 2.5, 6.0),
            Commands.waitSeconds(0.8),
            
            // Pickup from Coral Station
            Commands.runOnce(() -> System.out.println("[PickupCycles] Cycle 2: Picking up coral")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_CORAL_STATION, 2.5, 6.0),
            Commands.waitSeconds(0.8),
            
            // Cycle 3: Score on Reef Right
            Commands.runOnce(() -> System.out.println("[PickupCycles] Cycle 3: Scoring on Reef Right")),
            new AutoDriveCommand(drive, FieldConstants.REEF_RIGHT, 2.5, 6.0),
            Commands.waitSeconds(0.8),
            
            // Return home
            Commands.runOnce(() -> System.out.println("[PickupCycles] Returning to safe zone")),
            new AutoDriveCommand(drive, FieldConstants.BLUE_SAFE_ZONE, 2.0, 6.0),
            
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[PickupCycles] [OK] Complete - 3 pieces scored\n");
            }),
            
            // Hold position until autonomous ends - prevents default command from taking over
            Commands.run(() -> {
                drive.stop();
            }, drive).withName("HoldPosition")
        );
    }
}
