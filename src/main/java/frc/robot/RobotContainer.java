package frc.robot;

import java.io.File;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.auto.AutoAlgaeAuto;
import frc.robot.commands.auto.AutoAvoidCollision;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoComplexPath;
import frc.robot.commands.auto.AutoFigure8;
import frc.robot.commands.auto.AutoPickupAuto;
import frc.robot.commands.auto.AutoProcessorAuto;
import frc.robot.commands.auto.AutoSafeScore;
import frc.robot.commands.auto.AutoTripleScore;
import frc.robot.commands.auto.SequentialAuto;
import frc.robot.commands.auto.SimpleAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class RobotContainer {

  public DriveSubsystem m_DriveSubsystem;  // Not final - may fail to initialize
  private final CommandXboxController driverXbox;
  private final CommandXboxController m_operatorController;
  private LimelightVision limelightVision;
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize controller first
    driverXbox = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    m_operatorController = new CommandXboxController(1);
    
    // Initialize drive subsystem (wrapped in try-catch to allow robot to start even if it fails)
    try {
      m_DriveSubsystem = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "SWERVE"));
      System.out.println("[RobotContainer] DriveSubsystem initialized successfully");
      
      // Set initial robot position - START AT BLUE ALLIANCE
      m_DriveSubsystem.resetOdometry(FieldConstants.BLUE_ALLIANCE_START);
      System.out.println("[RobotContainer] Robot initialized at Blue Alliance start: (" + 
          String.format("%.2f, %.2f", FieldConstants.BLUE_ALLIANCE_START.getX(), 
          FieldConstants.BLUE_ALLIANCE_START.getY()) + ")");
    } catch (Exception e) {
      DriverStation.reportError("DriveSubsystem initialization failed: " + e.getMessage(), e.getStackTrace());
      System.err.println("[RobotContainer] CRITICAL: DriveSubsystem failed to initialize - robot will run in limited mode");
      e.printStackTrace();
      // m_DriveSubsystem will be null, checks will be needed in methods that use it
    }
    
    // Initialize vision (optional - won't crash if it fails)
    try {
      limelightVision = new LimelightVision();
      if (m_DriveSubsystem != null) {
        limelightVision.setDriveSubsystem(m_DriveSubsystem);
      }
    } catch (Exception e) {
      DriverStation.reportWarning("LimelightVision initialization failed: " + e.getMessage(), false);
      limelightVision = null;
    }

    // Configure commands and bindings
    configureDefaultCommand();
    configureBindings();
    
    // Setup dashboard
    setupSmartDashboard();
    setupAutonomousChooser();
    
    SmartDashboard.putBoolean("Robot/Container Initialized", true);
    SmartDashboard.putStringArray("Auto/Choices", new String[]{"Simple", "Sequential", "Balance"});
    SmartDashboard.putString("Auto/Choice", "Simple");
  }

  private void setupSmartDashboard() {
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
    
    // Alliance color selector
    SmartDashboard.putString("Alliance/Color", "Blue");
    
    // Field constants for debugging
    SmartDashboard.putNumber("Field/Length (m)", 17.54);
    SmartDashboard.putNumber("Field/Width (m)", 8.21);
    SmartDashboard.putNumber("Field/Blue Start X", FieldConstants.BLUE_ALLIANCE_START.getX());
    SmartDashboard.putNumber("Field/Blue Start Y", FieldConstants.BLUE_ALLIANCE_START.getY());
    SmartDashboard.putNumber("Field/Red Start X", FieldConstants.RED_ALLIANCE_START.getX());
    SmartDashboard.putNumber("Field/Red Start Y", FieldConstants.RED_ALLIANCE_START.getY());
  SmartDashboard.putNumber("Field/Hub Rest X", FieldConstants.HUB_REST_POSITION.getX());
  SmartDashboard.putNumber("Field/Hub Rest Y", FieldConstants.HUB_REST_POSITION.getY());
    
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

    // Computer key (dashboard) to lock and rest near hub (only if DriveSubsystem available)
    if (m_DriveSubsystem != null) {
      SmartDashboard.putData("Drive/Hub Rest", Commands.runOnce(() -> {
        m_DriveSubsystem.resetOdometry(FieldConstants.HUB_REST_POSITION);
        m_DriveSubsystem.lock();
        SmartDashboard.putString("Status/Last Action", "Hub Rest (Dashboard)");
      }, m_DriveSubsystem).withName("HubRest"));
    }
  }

  private void setupAutonomousChooser() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", Autos.doNothingAuto());
    
    // Only add drive-dependent autos if DriveSubsystem initialized successfully
    if (m_DriveSubsystem != null) {
      autoChooser.addOption("Drive Forward", Autos.driveForwardAuto(m_DriveSubsystem));
      
      // Basic scoring autos
      autoChooser.addOption("Simple (Reef)", new SimpleAuto(m_DriveSubsystem));
      autoChooser.addOption("Safe Score (1 piece)", new AutoSafeScore(m_DriveSubsystem));
      
      // Multi-piece autos
      autoChooser.addOption("Sequential (2 pieces)", new SequentialAuto(m_DriveSubsystem));
      autoChooser.addOption("Triple Score (3 pieces)", new AutoTripleScore(m_DriveSubsystem));
      
      // Specialized autos
      autoChooser.addOption("Avoid Collision", new AutoAvoidCollision(m_DriveSubsystem));
      autoChooser.addOption("Algae Removal", new AutoAlgaeAuto(m_DriveSubsystem));
      autoChooser.addOption("Processor Scoring", new AutoProcessorAuto(m_DriveSubsystem));
      autoChooser.addOption("Complex Path", new AutoComplexPath(m_DriveSubsystem));
      autoChooser.addOption("Balance", new AutoBalance(m_DriveSubsystem));
      autoChooser.addOption("Pickup Cycles", new AutoPickupAuto(m_DriveSubsystem));
      autoChooser.addOption("Figure 8 Pattern", new AutoFigure8(m_DriveSubsystem));
    } else {
      DriverStation.reportWarning("DriveSubsystem not initialized - limited autonomous options available", false);
    }
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDefaultCommand() {
    if (m_DriveSubsystem != null) {
      m_DriveSubsystem.setDefaultCommand(
        m_DriveSubsystem.driveCommand(
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX(),
          () -> -driverXbox.getRightX() * DriveConstants.ROTATION_SCALE
        ).withName("DefaultDrive")
      );
    }
  }

  private void configureBindings() {
    // Skip binding drive commands if DriveSubsystem failed to initialize
    if (m_DriveSubsystem == null) {
      DriverStation.reportWarning("DriveSubsystem not available - skipping drive button bindings", false);
      return;
    }
    
    // Speed control with bumpers
    driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {
      SmartDashboard.putString("Status/Last Action", "Speed increased");
    }));
    driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {
      SmartDashboard.putString("Status/Last Action", "Speed decreased");
    }));

    // Heading reset (POV Down)
    driverXbox.povDown().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.seedForwards();
      SmartDashboard.putString("Status/Last Action", "Heading Reset");
    }));
    
    // Zero gyro and sync modules (POV Up)
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
    
    // POV Right - Lock wheels
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

    // Controller key to lock at hub rest position (press START)
    driverXbox.start().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.resetOdometry(FieldConstants.HUB_REST_POSITION);
      m_DriveSubsystem.lock();
      SmartDashboard.putString("Status/Last Action", "Hub Rest (Start)");
    }, m_DriveSubsystem));

    // Full speed mode (right trigger) - Remove duplicate deadband
    driverXbox.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD)
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "FULL SPEED")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "Normal")))
      .whileTrue(
        m_DriveSubsystem.driveCommand(
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX(),
          () -> -driverXbox.getRightX() * DriveConstants.FULL_SPEED_ROTATION_SCALE
        ).withName("FullSpeedDrive")
      );
    
    // Precision mode (left trigger) - Remove duplicate deadband
    driverXbox.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD)
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "PRECISION")))
      .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Status/Speed Mode", "Normal")))
      .whileTrue(
        m_DriveSubsystem.driveCommand(
          () -> -driverXbox.getLeftY() * DriveConstants.PRECISION_MULTIPLIER,
          () -> -driverXbox.getLeftX() * DriveConstants.PRECISION_MULTIPLIER,
          () -> -driverXbox.getRightX() * DriveConstants.PRECISION_ROTATION_SCALE
        ).withName("PrecisionDrive")
      );
    
    // Test auto with A button
    driverXbox.a()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Last Action", "Running Auto Test")))
      .whileTrue(
        Commands.defer(this::getAutonomousCommand, m_DriveSubsystem != null ? Set.of(m_DriveSubsystem) : Set.of())
      );
    
    // Initialize status
    SmartDashboard.putString("Status/Speed Mode", "Normal");
    SmartDashboard.putString("Status/Last Action", "Ready");

    // Operator diagnostics (only if DriveSubsystem available)
    if (m_DriveSubsystem != null) {
      m_operatorController.a().onTrue(Commands.runOnce(m_DriveSubsystem::printEncoderOffsets));
      m_operatorController.b().onTrue(Commands.runOnce(m_DriveSubsystem::diagnoseGyro));
      m_operatorController.x().onTrue(Commands.runOnce(m_DriveSubsystem::diagnoseMotorInversions));
    }
  }
  
  /**
   * Get the autonomous command to run.
   * Always ensures robot starts at correct field position.
   */
  public Command getAutonomousCommand() {
    // If DriveSubsystem failed to initialize, return do-nothing command
    if (m_DriveSubsystem == null) {
      DriverStation.reportWarning("Cannot run autonomous - DriveSubsystem not initialized", false);
      return Autos.doNothingAuto();
    }
    
    // Get selected starting position from dashboard or default to blue
    String allianceColor = SmartDashboard.getString("Alliance/Color", "Blue");
    
    Pose2d startingPose = allianceColor.equals("Red") ? 
        FieldConstants.RED_ALLIANCE_START : 
        FieldConstants.BLUE_ALLIANCE_START;
    
    // Reset to correct starting position for selected alliance
    m_DriveSubsystem.stop();
    m_DriveSubsystem.zeroGyro();
    m_DriveSubsystem.resetOdometry(startingPose);
    
    // Get command from SendableChooser instead of string
    Command selectedAuto = autoChooser.getSelected();
    System.out.println("[RobotContainer] Selected autonomous: " + selectedAuto.getName());
    
    return selectedAuto != null ? selectedAuto : Autos.doNothingAuto();
  }

  /**
   * Called when teleop starts.
   */
  public void teleopInit() {
    if (m_DriveSubsystem == null) {
      DriverStation.reportWarning("DriveSubsystem not available for teleop", false);
      return;
    }
    
    // Reset to Blue Alliance start at teleop begin
    m_DriveSubsystem.resetOdometry(FieldConstants.BLUE_ALLIANCE_START);
    System.out.println("Teleop initialized - odometry reset to Blue Alliance start");
  }  
}
