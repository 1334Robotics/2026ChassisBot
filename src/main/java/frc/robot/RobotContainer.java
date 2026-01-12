package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_DriveSubsystem;
  private final CommandXboxController driverXbox;
  private Command driveFieldOrientedDirectAngle;

  public RobotContainer() {

    driverXbox = new CommandXboxController(0);

    m_DriveSubsystem =
        new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Default drive command
    driveFieldOrientedDirectAngle = 
        m_DriveSubsystem.driveCommand(
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), 0.1),
            () -> driverXbox.getRightX(),
            () -> driverXbox.getRightY()
        );

    m_DriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);

    configureBindings();
  }

  private void configureBindings() {

    driverXbox.povDown()
        .onTrue(Commands.runOnce(() -> m_DriveSubsystem.seedForwards()));

    driverXbox.a()
        .whileTrue(
            m_DriveSubsystem.driveCommand(
                () -> 0.1,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0
            )
        );
  }

  public Command getAutonomousCommand() {
    return m_DriveSubsystem
        .driveCommand(
            () -> -0.6,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0
        )
        .withTimeout(3.0);
  }

  public void teleopSequence() {
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
  }
}
