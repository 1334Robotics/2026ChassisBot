package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous command that follows a PathPlanner path.
 * Tries to load 'ExamplePath' from deploy/pathplanner/paths/.
 * Falls back to an on-the-fly path if the file doesn't exist.
 */
public class AutoPathes extends SequentialCommandGroup {
    public AutoPathes(DriveSubsystem drive) {
        Command followCommand = buildFollowCommand(drive);

        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== AUTO PATHES ==========");
                System.out.println("Following PathPlanner path");
                System.out.println("=================================\n");
            }),

            // Small settle delay
            Commands.waitSeconds(0.1),

            // Follow the path
            followCommand,

            // Stop when done
            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[AutoPathes] Path complete. Robot stopped and locked.");
            })
        );
    }

    /**
     * Build the follow command. Tries to load from file first, falls back to on-the-fly path.
     */
    private Command buildFollowCommand(DriveSubsystem drive) {
        // Try loading from file first
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("ExamplePath");
            System.out.println("[AutoPathes] Loaded 'ExamplePath' from file.");

            // Reset pose to path start before following
            return Commands.sequence(
                Commands.runOnce(() -> {
                    try {
                        path.getStartingHolonomicPose().ifPresent(pose -> {
                            drive.resetOdometry(pose);
                            System.out.println("[AutoPathes] Reset pose to path start: " + pose);
                        });
                    } catch (Exception e) {
                        System.err.println("[AutoPathes] Could not reset pose: " + e.getMessage());
                    }
                }),
                Commands.waitSeconds(0.1),
                AutoBuilder.followPath(path)
            );
        } catch (Exception e) {
            System.out.println("[AutoPathes] 'ExamplePath' not found, using on-the-fly path.");
        }

        // Fallback: create an on-the-fly path from current position forward
        try {
            Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.kZero);
            Pose2d end   = new Pose2d(3.0, 3.0, Rotation2d.kZero);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);
            PathConstraints constraints = new PathConstraints(
                2.0, 2.0,    // max velocity (m/s), max acceleration (m/s²)
                Math.PI, Math.PI  // max angular velocity, max angular acceleration
            );
            PathPlannerPath onTheFlyPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,  // ideal starting state (null = use current)
                new GoalEndState(0.0, end.getRotation())  // stop at end
            );

            System.out.println("[AutoPathes] Created on-the-fly path: (1,1) -> (3,3)");

            return Commands.sequence(
                Commands.runOnce(() -> {
                    drive.resetOdometry(start);
                    System.out.println("[AutoPathes] Reset pose to on-the-fly start: " + start);
                }),
                Commands.waitSeconds(0.1),
                AutoBuilder.followPath(onTheFlyPath)
            );
        } catch (Exception e) {
            System.err.println("[AutoPathes] ERROR: Could not create on-the-fly path: " + e.getMessage());
            e.printStackTrace();
            return Commands.none();
        }
    }
}