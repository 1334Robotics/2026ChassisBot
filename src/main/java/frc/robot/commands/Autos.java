package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Utility class containing autonomous routines for FIRST Rebuilt 2025.
 * 
 * Game: FIRST Reefscape Rebuilt
 * 
 * Scoring Opportunities:
 * - Reef Scoring: 3 points per coral (3 branches: Front, Left, Right)
 * - Processor Scoring: 4 points per coral (higher value)
 * - Algae Removal: Bonus points (variable)
 * 
 * Field Layout:
 * - Field Size: 16.54m × 8.07m (54.27' × 26.47')
 * - Reef Center: (8.27, 4.035) - Center obstacle
 * - Coral Stations: Alliance walls for pickup
 * - Processor Stations: Side stations for bonus scoring
 * 
 * Available Auto Routines:
 * - Simple: 1 coral (3 pts) - Safe and reliable
 * - Triple Score: 3 corals (9 pts) - Multi-branch scoring
 * - Complex Path: 3 corals (9 pts) - With pickup cycles
 * - Pickup Cycles: 3 corals (9 pts) - Rapid cycling practice
 * - Sequential: 1 coral (3 pts) - Multi-waypoint demo
 * - Processor: 1 coral (4 pts) - Processor bonus scoring
 * - Algae Removal: Variable points - Field control
 * - Figure 8: Navigation practice only
 * - Balance/Center: Positioning practice only
 * - Collision Avoid: Safe pathing practice
 */
public final class Autos {

  /**
   * Do nothing autonomous - robot stays still.
   */
  public static Command doNothingAuto() {
    return Commands.none().withName("Do Nothing");
  }

  /**
   * Drive forward autonomous routine.
   * Robot drives forward at a constant speed for a set duration.
   */
  public static Command driveForwardAuto(DriveSubsystem driveSubsystem) {
    return Commands.sequence(
        // Drive forward at 50% speed for 3 seconds
        Commands.run(() -> driveSubsystem.driveFieldOriented(
            new ChassisSpeeds(2.25, 0, 0)), driveSubsystem)
            .withTimeout(3),
        // Stop the robot
        Commands.runOnce(() -> driveSubsystem.stop(), driveSubsystem)
    ).withName("Drive Forward");
  }

  // Private constructor to prevent instantiation
  private Autos() {
    throw new UnsupportedOperationException("Utility class - do not instantiate");
  }
}