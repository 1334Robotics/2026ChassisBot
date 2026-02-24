package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPathes extends SequentialCommandGroup{
    public AutoPathes(DriveSubsystem drive){
        addCommands(
            Commands.runOnce(() -> {
                System.out.println("\n========== AUTO PATHES ==========");
                System.out.println("Following PathPlanner 'ExamplePath'");
                System.out.println("End Position: (1.141, 3.867)");
                System.out.println("End Heading: 176.644°");
                System.out.println("========================================\n");
            }),
            Commands.runOnce(() -> drive.stop()),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> System.out.println("[AutoPathes] Starting path following...")),
            getPathCommand(),

            Commands.runOnce(() -> {
                drive.stop();
                drive.lock();
                System.out.println("[AutoPathes] Path following complete. Robot stopped and locked.");
            }),
            Commands.run(() -> drive.stop(), drive).withName("HoldPosition")
            
        );

    }
    private Command getPathCommand(){
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("ExamplePath");
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            System.err.println("[ExamplePath] ERROR: Could not load path file!");
            System.err.println("  Make sure 'ExamplePath.path' exists in deploy/pathplanner/paths/");
            e.printStackTrace();
            return Commands.none();
        }
    }
}