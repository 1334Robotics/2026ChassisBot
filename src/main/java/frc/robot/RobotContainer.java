package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class RobotContainer {

  private final DriveSubsystem m_DriveSubsystem;
  private final CommandXboxController driverXbox;
  private final LimelightVision limelightVision;
  
  // Autonomous chooser - uses Supplier to create fresh commands each time
  private final SendableChooser<Supplier<Command>> autoChooser;
  
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

    // Setup autonomous chooser
    autoChooser = new SendableChooser<>();
    configureAutoChooser();

    configureDefaultCommand();
    configureBindings();
  }

  private void configureAutoChooser() {
    // Add autonomous options using Suppliers (factories) so fresh commands are created each time
    autoChooser.setDefaultOption("Simple Forward", () -> Autos.simpleForwardAuto(m_DriveSubsystem));
    autoChooser.addOption("Forward and Back", () -> Autos.forwardAndBackAuto(m_DriveSubsystem));
    autoChooser.addOption("Square Path", () -> Autos.squarePathAuto(m_DriveSubsystem));
    autoChooser.addOption("Figure 8", () -> Autos.figureEightAuto(m_DriveSubsystem));
    autoChooser.addOption("Spin in Place", () -> Autos.spinInPlaceAuto(m_DriveSubsystem));
    autoChooser.addOption("S-Curve", () -> Autos.sCurveAuto(m_DriveSubsystem));
    autoChooser.addOption("Mobility", () -> Autos.mobilityAuto(m_DriveSubsystem));
    autoChooser.addOption("Out and Back", () -> Autos.outAndBackAuto(m_DriveSubsystem));
    autoChooser.addOption("Strafe Test", () -> Autos.strafeTestAuto(m_DriveSubsystem));
    autoChooser.addOption("Diagonal Drive", () -> Autos.diagonalDriveAuto(m_DriveSubsystem));
    autoChooser.addOption("Do Nothing", () -> Autos.doNothingAuto());
    
    // Put chooser on dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // ==================== DEFAULT COMMAND ====================

  private void configureDefaultCommand() {
    // Simple default drive command
    m_DriveSubsystem.setDefaultCommand(
      m_DriveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DEADBAND) * 0.5,
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DEADBAND) * 0.5,
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), DEADBAND) * 0.3,
        () -> 0.0
      )
    );
  }

  // ==================== BUTTON BINDINGS ====================

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
    
    // A button: Run selected auto (for testing in teleop)
    // This properly gets a fresh command and runs it
    driverXbox.a()
        .whileTrue(
            Commands.defer(() -> {
                Supplier<Command> selectedAutoSupplier = autoChooser.getSelected();
                if (selectedAutoSupplier != null) {
                    return selectedAutoSupplier.get();
                }
                return Commands.none();
            }, java.util.Set.of(m_DriveSubsystem))
        );
  }

  // ==================== AUTONOMOUS COMMAND ====================

  /**
   * Returns a fresh autonomous command each time it's called.
   */
  public Command getAutonomousCommand() {
    Supplier<Command> selectedAutoSupplier = autoChooser.getSelected();
    if (selectedAutoSupplier != null) {
      return selectedAutoSupplier.get();
    }
    return Commands.none();
  }

  public void teleopSequence() {
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
  }
}
