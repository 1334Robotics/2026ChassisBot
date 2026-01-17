package frc.robot;

import java.io.File;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.SequentialAuto;
import frc.robot.commands.auto.SimpleAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class RobotContainer {

  private final DriveSubsystem m_DriveSubsystem;
  private final CommandXboxController driverXbox;
  private final CommandXboxController m_operatorController;
  private LimelightVision limelightVision;
  
  // Autonomous chooser
  private final SendableChooser<Supplier<Command>> autoChooser;
  
  // Speed control - adjustable with bumpers
  private double speedMultiplier = DriveConstants.DEFAULT_SPEED_MULTIPLIER;

  public RobotContainer() {
    // Initialize controller first
    driverXbox = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    m_operatorController = new CommandXboxController(1);
    
    // Initialize drive subsystem
    m_DriveSubsystem = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "SWERVE"));
    
    // Initialize vision (optional - won't crash if it fails)
    try {
      limelightVision = new LimelightVision();
      limelightVision.setDriveSubsystem(m_DriveSubsystem);
    } catch (Exception e) {
      DriverStation.reportWarning("LimelightVision initialization failed: " + e.getMessage(), false);
      limelightVision = null;
    }

    // Set initial robot position - START AT ORIGIN FOR TESTING
    m_DriveSubsystem.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.kZero));

    // Setup autonomous chooser
    autoChooser = new SendableChooser<>();
    configureAutoChooser();

    // Configure commands and bindings
    configureDefaultCommand();
    configureBindings();
    
    // Setup dashboard
    setupSmartDashboard();
    
    SmartDashboard.putBoolean("Robot/Container Initialized", true);
    
    SmartDashboard.putStringArray("Auto/Choices", new String[]{"Simple", "Sequential", "Balance"});
    SmartDashboard.putString("Auto/Choice", "Simple");
  }

  private void setupSmartDashboard() {
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
    
    // Controller info
    SmartDashboard.putNumber("Controller/Deadband", DriveConstants.DEADBAND);
    updateSpeedDisplay();
    
    // Control instructions
    SmartDashboard.putString("Controls/Left Stick", "Move Robot");
    SmartDashboard.putString("Controls/Right Stick X", "Rotate Robot");
    SmartDashboard.putString("Controls/Right Bumper (R1)", "Increase Speed (+10%)");
    SmartDashboard.putString("Controls/Left Bumper (L1)", "Decrease Speed (-10%)");
    SmartDashboard.putString("Controls/Right Trigger", "Full Speed Mode");
    SmartDashboard.putString("Controls/Left Trigger", "Precision Mode");
    SmartDashboard.putString("Controls/A Button", "Test Auto");
    SmartDashboard.putString("Controls/B Button", "Reset to Blue Start");
    SmartDashboard.putString("Controls/X Button", "Reset to Red Start");
    SmartDashboard.putString("Controls/Y Button", "Reset to Center");
    SmartDashboard.putString("Controls/POV Down", "Reset Heading");
    SmartDashboard.putString("Controls/POV Up", "Zero Gyro & Sync Modules");
  }
  
  private void updateSpeedDisplay() {
    SmartDashboard.putNumber("Controller/Speed Multiplier", speedMultiplier);
    SmartDashboard.putNumber("Controller/Speed %", Math.round(speedMultiplier * 100));
  }
  
  private void increaseSpeed() {
    speedMultiplier = Math.min(speedMultiplier + DriveConstants.SPEED_INCREMENT, DriveConstants.MAX_SPEED_MULTIPLIER);
    speedMultiplier = Math.round(speedMultiplier * 10.0) / 10.0;
    updateSpeedDisplay();
    SmartDashboard.putString("Status/Last Action", "Speed: " + Math.round(speedMultiplier * 100) + "%");
  }
  
  private void decreaseSpeed() {
    speedMultiplier = Math.max(speedMultiplier - DriveConstants.SPEED_INCREMENT, DriveConstants.MIN_SPEED_MULTIPLIER);
    speedMultiplier = Math.round(speedMultiplier * 10.0) / 10.0;
    updateSpeedDisplay();
    SmartDashboard.putString("Status/Last Action", "Speed: " + Math.round(speedMultiplier * 100) + "%");
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", () -> Autos.doNothingAuto());
    autoChooser.addOption("Simple Forward", () -> Autos.simpleForwardAuto(m_DriveSubsystem));
    autoChooser.addOption("Forward and Back", () -> Autos.forwardAndBackAuto(m_DriveSubsystem));
    autoChooser.addOption("Square Path", () -> Autos.squarePathAuto(m_DriveSubsystem));
    autoChooser.addOption("Figure 8", () -> Autos.figureEightAuto(m_DriveSubsystem));
    autoChooser.addOption("Spin in Place", () -> Autos.spinInPlaceAuto(m_DriveSubsystem));
    autoChooser.addOption("S-Curve", () -> Autos.sCurveAuto(m_DriveSubsystem));
    autoChooser.addOption("Mobility", () -> Autos.mobilityAuto(m_DriveSubsystem));
    autoChooser.addOption("Out and Back", () -> Autos.outAndBackAuto(m_DriveSubsystem));
    autoChooser.addOption("Strafe Test", () -> Autos.strafeTestAuto(m_DriveSubsystem));
    autoChooser.addOption("Diagonal Drive", () -> Autos.diagonalDriveAuto(m_DriveSubsystem));
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDefaultCommand() {
    m_DriveSubsystem.setDefaultCommand(
      m_DriveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriveConstants.DEADBAND) * speedMultiplier,
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriveConstants.DEADBAND) * speedMultiplier,
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriveConstants.DEADBAND) * speedMultiplier * DriveConstants.ROTATION_SCALE
      ).withName("DefaultDrive")
    );
  }

  private void configureBindings() {
    // Speed control with bumpers
    driverXbox.rightBumper().onTrue(Commands.runOnce(this::increaseSpeed));
    driverXbox.leftBumper().onTrue(Commands.runOnce(this::decreaseSpeed));

    // Heading reset (POV Down)
    driverXbox.povDown().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.seedForwards();
      SmartDashboard.putString("Status/Last Action", "Heading Reset");
    }));
    
    // Zero gyro and sync modules (POV Up) - fixes spinning issues
    driverXbox.povUp().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.zeroGyro();
      m_DriveSubsystem.synchronizeModuleEncoders();
      SmartDashboard.putString("Status/Last Action", "Gyro & Modules Zeroed");
    }));
    
    // Print encoder offsets for calibration (POV Left)
    driverXbox.povLeft().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.printEncoderOffsets();
      SmartDashboard.putString("Status/Last Action", "Printed Encoder Offsets - Check Console");
    }));
    
    // POV Right - Lock wheels in X pattern (prevents pushing)
    driverXbox.povRight().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.lock();
      SmartDashboard.putString("Status/Last Action", "Wheels Locked");
    }));

    // Position resets
    driverXbox.b().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.resetOdometry(FieldConstants.BLUE_ALLIANCE_START);
      SmartDashboard.putString("Status/Last Action", "Reset to Blue Start");
    }));
    
    driverXbox.x().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.resetOdometry(FieldConstants.RED_ALLIANCE_START);
      SmartDashboard.putString("Status/Last Action", "Reset to Red Start");
    }));
    
    driverXbox.y().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.resetOdometry(FieldConstants.CENTER_START);
      SmartDashboard.putString("Status/Last Action", "Reset to Center");
    }));

    // Full speed mode (right trigger)
    driverXbox.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD)
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "FULL SPEED")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "Normal")))
      .whileTrue(
        m_DriveSubsystem.driveCommand(
          () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriveConstants.DEADBAND),
          () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriveConstants.DEADBAND),
          () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriveConstants.DEADBAND) * DriveConstants.FULL_SPEED_ROTATION_SCALE
        ).withName("FullSpeedDrive")
      );
    
    // Precision mode (left trigger)
    driverXbox.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD)
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "PRECISION")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "Normal")))
      .whileTrue(
        m_DriveSubsystem.driveCommand(
          () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriveConstants.DEADBAND) * DriveConstants.PRECISION_MULTIPLIER,
          () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriveConstants.DEADBAND) * DriveConstants.PRECISION_MULTIPLIER,
          () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriveConstants.DEADBAND) * DriveConstants.PRECISION_ROTATION_SCALE
        ).withName("PrecisionDrive")
      );
    
    // Test auto with A button
    driverXbox.a()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Last Action", "Running Auto Test")))
      .whileTrue(
        Commands.defer(this::getSelectedAutoCommand, Set.of(m_DriveSubsystem))
      );
    
    // Initialize status
    SmartDashboard.putString("Status/Speed Mode", "Normal");
    SmartDashboard.putString("Status/Last Action", "Ready");

    // Operator diagnostics
    m_operatorController.a().onTrue(Commands.runOnce(m_DriveSubsystem::printEncoderOffsets));
    m_operatorController.b().onTrue(Commands.runOnce(m_DriveSubsystem::diagnoseGyro));
  }
  
  /**
   * Get the currently selected autonomous command.
   * Always ensures robot starts at origin (0, 0).
   */
  private Command getSelectedAutoCommand() {
    try {
      Supplier<Command> selectedSupplier = autoChooser.getSelected();
      if (selectedSupplier != null) {
        Command cmd = selectedSupplier.get();
        return cmd != null ? cmd : Commands.none();
      }
    } catch (Exception e) {
      DriverStation.reportError("Error getting auto command: " + e.getMessage(), false);
    }
    return Commands.none();
  }

  /**
   * Get the autonomous command to run.
   * Always ensures robot starts at origin (0, 0).
   */
  public Command getAutonomousCommand() {
    // Reset to origin before any autonomous
    m_DriveSubsystem.stop();
    m_DriveSubsystem.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    
    String autoChoice = SmartDashboard.getString("Auto/Choice", "Simple");
    System.out.println("[RobotContainer] Selected autonomous: " + autoChoice);
    
    return switch(autoChoice) {
        case "Sequential" -> new SequentialAuto(m_DriveSubsystem);
        case "Balance" -> new AutoBalance(m_DriveSubsystem);
        default -> new SimpleAuto(m_DriveSubsystem);
    };
  }

  /**
   * Called when teleop starts.
   */
  public void teleopInit() {
    // Reset to origin at start of teleop
    m_DriveSubsystem.resetOdometry(new Pose2d());
    System.out.println("Teleop initialized - odometry reset");
  }
}
