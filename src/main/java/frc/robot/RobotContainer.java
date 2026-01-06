package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer { 
  private final SwerveSubsystem m_SwerveSubsystem;
  private final CommandXboxController driverXbox;

  public RobotContainer() {
    m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    driverXbox = new CommandXboxController(0);

    configureBindings();

    // Shuffleboard (organized tabs)
    Shuffleboard.getTab("Diagnostics").add("Command Scheduler", CommandScheduler.getInstance());
    
    // SmartDashboard (simple key-value)
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Swerve Subsystem", m_SwerveSubsystem);
  }

  private void configureBindings() {
    // Your button bindings
    driverXbox.povDown().onTrue(Commands.runOnce(m_SwerveSubsystem::seedForwards, m_SwerveSubsystem));

    // Set default command
    m_SwerveSubsystem.setDefaultCommand(
        m_SwerveSubsystem.driveCommand(
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX(),
            () -> -driverXbox.getRightX(),
            () -> -driverXbox.getRightY()
        )
    );
  }

  // Remove or empty teleopSequence() - don't add Shuffleboard stuff here!
  public void teleopSequence() {
    // Don't add Shuffleboard widgets here - this runs every loop!
  }

  public void updateDashboard() {
    // These update every loop - put changing values here
    SmartDashboard.putNumber("Robot X", m_SwerveSubsystem.getPose().getX());
    SmartDashboard.putNumber("Robot Y", m_SwerveSubsystem.getPose().getY());
    SmartDashboard.putNumber("Robot Heading", m_SwerveSubsystem.getPose().getRotation().getDegrees());
  }
}