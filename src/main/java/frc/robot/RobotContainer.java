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
import frc.robot.subsystems.LimelightVision;

public class RobotContainer {

  private final DriveSubsystem m_DriveSubsystem;
  private final CommandXboxController driverXbox;
  private final LimelightVision limelightVision;
  
  // Deadband for joystick
  private static final double DEADBAND = 0.1;

  public RobotContainer() {
    driverXbox = new CommandXboxController(0);

    m_DriveSubsystem =
        new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "SWERVE"));

    limelightVision = new LimelightVision();
    
    // Link vision system to drive subsystem for automatic pose updates
    limelightVision.setDriveSubsystem(m_DriveSubsystem);

    // Set initial robot position to be inside the field
    m_DriveSubsystem.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START);

    configureDefaultCommand();
    configureBindings();
  }

  private void configureDefaultCommand() {
    // Simple default drive command
    m_DriveSubsystem.setDefaultCommand(
      m_DriveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DEADBAND) * 0.5,  // Forward/back at 50%
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DEADBAND) * 0.5,  // Strafe at 50%
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), DEADBAND) * 0.3, // Rotate at 30%
        () -> 0.0
      )
    );
  }

  private void configureBindings() {
    // POV Down: Reset heading
    driverXbox.povDown()
        .onTrue(Commands.runOnce(() -> m_DriveSubsystem.seedForwards()));

    // B button: Reset to blue start
    driverXbox.b()
        .onTrue(Commands.runOnce(() -> 
            m_DriveSubsystem.resetOdometry(Constants.FieldConstants.BLUE_ALLIANCE_START)));
    
    // X button: Reset to red start
    driverXbox.x()
        .onTrue(Commands.runOnce(() -> 
            m_DriveSubsystem.resetOdometry(Constants.FieldConstants.RED_ALLIANCE_START)));
    
    // Y button: Reset to center
    driverXbox.y()
        .onTrue(Commands.runOnce(() -> 
            m_DriveSubsystem.resetOdometry(Constants.FieldConstants.CENTER_START)));

    // Right Trigger: Full speed mode
    driverXbox.rightTrigger(0.5)
        .whileTrue(
            m_DriveSubsystem.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getRightX(), DEADBAND) * 0.5,
                () -> 0.0
            )
        );
    
    // Left Trigger: Slow precision mode
    driverXbox.leftTrigger(0.5)
        .whileTrue(
            m_DriveSubsystem.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DEADBAND) * 0.2,
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DEADBAND) * 0.2,
                () -> MathUtil.applyDeadband(driverXbox.getRightX(), DEADBAND) * 0.1,
                () -> 0.0
            )
        );
  }

  public Command getAutonomousCommand() {
    return m_DriveSubsystem
        .driveCommand(
            () -> 0.3,
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
