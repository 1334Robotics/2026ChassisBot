package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer { 
  private final SwerveSubsystem m_SwerveSubsystem;
  private final CommandXboxController driverXbox;
  private final Command driveFieldOrientedDirectAngle;

  public RobotContainer() {
    driverXbox = new CommandXboxController(0); 

    m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Now that m_DriveSubsystem is initialized, we can call driveCommand
    driveFieldOrientedDirectAngle = m_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), 0.1),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Set default command
    m_SwerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);

  /*  configureBindings();
  }

  private void configureBindings() {
    // e.g., driverXbox.a().onTrue(Commands.print("A pressed"));
    driverXbox.povDown().onTrue(Commands.runOnce(() -> m_SwerveSubsystem.seedForwards()));
    driverXbox.a().whileTrue(m_SwerveSubsystem.driveCommand(() -> 0.1, () -> 0.0, () -> 0.0, () -> 0.0));
  }

  public void teleopSequence() {
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());*/
  }
}