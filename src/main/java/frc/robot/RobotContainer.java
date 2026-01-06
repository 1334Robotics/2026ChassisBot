package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    // Add to Shuffleboard ONCE here, not in a periodic method
    Shuffleboard.getTab("Diagnostics").add("Command Scheduler", CommandScheduler.getInstance());
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
}