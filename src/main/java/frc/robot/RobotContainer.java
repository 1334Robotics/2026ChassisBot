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
import frc.robot.commands.auto.AutoPathes;
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
import com.pathplanner.lib.auto.AutoBuilder;
@SuppressWarnings("unused")  // Many commands and constants are defined but not used yet as we build out functionality incrementally
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
      
      // Set initial robot position based on alliance reported by Driver Station
      Pose2d startPose = getAllianceStartPose();
      m_DriveSubsystem.resetOdometry(startPose);
      System.out.println("[RobotContainer] Robot initialized at " +
          (isRedAlliance() ? "Red" : "Blue") + " Alliance start: (" +
          String.format("%.2f, %.2f", startPose.getX(), startPose.getY()) + ")");
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
    
    // Control mode indicator
    SmartDashboard.putString("Controls/Mode", "Keyboard: Arrows+QE | Xbox: Sticks (both work)");
    
    // Field constants for debugging
    SmartDashboard.putNumber("Field/Length (m)", 17.54);
    SmartDashboard.putNumber("Field/Width (m)", 8.21);
    SmartDashboard.putNumber("Field/Blue Start X", FieldConstants.BLUE_ALLIANCE_START.getX());
    SmartDashboard.putNumber("Field/Blue Start Y", FieldConstants.BLUE_ALLIANCE_START.getY());
    SmartDashboard.putNumber("Field/Red Start X", FieldConstants.RED_ALLIANCE_START.getX());
    SmartDashboard.putNumber("Field/Red Start Y", FieldConstants.RED_ALLIANCE_START.getY());
  SmartDashboard.putNumber("Field/Hub Rest X", FieldConstants.HUB_REST_POSITION.getX());
  SmartDashboard.putNumber("Field/Hub Rest Y", FieldConstants.HUB_REST_POSITION.getY());
    
    // Control instructions - Xbox Controller
    SmartDashboard.putString("Xbox/Left Stick", "Move Robot");
    SmartDashboard.putString("Xbox/Right Stick X", "Rotate Robot");
    SmartDashboard.putString("Xbox/Right Bumper (R1)", "Increase Speed (+10%)");
    SmartDashboard.putString("Xbox/Left Bumper (L1)", "Decrease Speed (-10%)");
    SmartDashboard.putString("Xbox/Right Trigger", "Full Speed Mode");
    SmartDashboard.putString("Xbox/Left Trigger", "Precision Mode");
    SmartDashboard.putString("Xbox/A Button", "Test Auto");
    SmartDashboard.putString("Xbox/B Button", "Reset to Blue Start");
    SmartDashboard.putString("Xbox/X Button", "Reset to Red Start");
    SmartDashboard.putString("Xbox/Y Button", "Reset to Center");
    SmartDashboard.putString("Xbox/POV Down", "Reset Heading");
    SmartDashboard.putString("Xbox/POV Up", "Zero Gyro & Sync Modules");
    
    // Keyboard instructions
    SmartDashboard.putString("Keyboard/W", "Forward");
    SmartDashboard.putString("Keyboard/S", "Backward");
    SmartDashboard.putString("Keyboard/A", "Strafe Left");
    SmartDashboard.putString("Keyboard/D", "Strafe Right");
    SmartDashboard.putString("Keyboard/Q", "Rotate Left (instant)");
    SmartDashboard.putString("Keyboard/F", "Rotate Right (instant)");
    SmartDashboard.putString("Keyboard/E", "Rotate Left (smooth)");
    SmartDashboard.putString("Keyboard/R", "Rotate Right (smooth)");
    SmartDashboard.putString("Keyboard/Arrows", "Alternative WASD");

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
    // Use PathPlanner's AutoBuilder if configured, otherwise fall back to manual chooser
    if (m_DriveSubsystem != null && AutoBuilder.isConfigured()) {
      // PathPlanner auto chooser - reads autos from src/main/deploy/pathplanner/autos/
      autoChooser = AutoBuilder.buildAutoChooser();
      System.out.println("[RobotContainer] PathPlanner AutoBuilder chooser configured");
    } else {
      // Fallback: manual auto chooser using deferred commands (so they can be re-run)
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("Do Nothing", Autos.doNothingAuto());
      
      if (m_DriveSubsystem != null) {
        Set<edu.wpi.first.wpilibj2.command.Subsystem> driveReqs = Set.of(m_DriveSubsystem);
        autoChooser.addOption("Drive Forward",
            Commands.defer(() -> Autos.driveForwardAuto(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Simple (Reef)",
            Commands.defer(() -> new SimpleAuto(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Safe Score (1 piece)",
            Commands.defer(() -> new AutoSafeScore(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Sequential (2 pieces)",
            Commands.defer(() -> new SequentialAuto(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Triple Score (3 pieces)",
            Commands.defer(() -> new AutoTripleScore(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Avoid Collision",
            Commands.defer(() -> new AutoAvoidCollision(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Algae Removal",
            Commands.defer(() -> new AutoAlgaeAuto(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Processor Scoring",
            Commands.defer(() -> new AutoProcessorAuto(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Complex Path",
            Commands.defer(() -> new AutoComplexPath(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Balance",
            Commands.defer(() -> new AutoBalance(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Pickup Cycles",
            Commands.defer(() -> new AutoPickupAuto(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Figure 8 Pattern",
            Commands.defer(() -> new AutoFigure8(m_DriveSubsystem), driveReqs));
        autoChooser.addOption("Auto Pathes (PathPlanner)",
            Commands.defer(() -> new AutoPathes(m_DriveSubsystem), driveReqs));
      } else {
        DriverStation.reportWarning("DriveSubsystem not initialized - limited autonomous options available", false);
      }
    }
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDefaultCommand() {
    if (m_DriveSubsystem != null) {
      m_DriveSubsystem.setDefaultCommand(
        m_DriveSubsystem.driveCommand(
          // Forward/back: W/S keys (sim) or Xbox left stick Y (real robot) — pick larger magnitude
          () -> {
            double kb = -Input.getTranslationY();
            double xbox = -driverXbox.getLeftY();
            return (Math.abs(kb) > Math.abs(xbox)) ? kb : xbox;
          },
          // Strafe: A/D keys (sim) or Xbox left stick X (real robot)
          () -> {
            double kb = -Input.getTranslationX();
            double xbox = -driverXbox.getLeftX();
            return (Math.abs(kb) > Math.abs(xbox)) ? kb : xbox;
          },
          // Rotation: E/R axis + Q/F buttons (sim) or Xbox right stick X (real robot)
          // Keyboard gives 0 or ±1 (buttons) or slow ramp (axis), so don't pre-scale here.
          // DriveSubsystem.driveCommand applies cube scaling internally, which is enough.
          // Only apply ROTATION_SCALE to Xbox stick (which is analog and needs dampening).
          () -> {
            double kb = -Input.getRotation();
            double xbox = -driverXbox.getRightX() * DriveConstants.ROTATION_SCALE;
            return (Math.abs(kb) > Math.abs(xbox)) ? kb : xbox;
          }
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
    
    // Back button - EMERGENCY STOP: instantly stop all motors and lock wheels
    driverXbox.back().onTrue(Commands.runOnce(() -> {
      CommandScheduler.getInstance().cancelAll();
      m_DriveSubsystem.stop();
      m_DriveSubsystem.lock();
      SmartDashboard.putString("Status/Last Action", "EMERGENCY STOP");
      System.out.println("[RobotContainer] EMERGENCY STOP triggered");
    }, m_DriveSubsystem));
    
    // Speed control with bumpers — persistently adjusts drive speed
    driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.increaseSpeed();
      SmartDashboard.putString("Status/Last Action",
          "Speed: " + String.format("%.0f%%", m_DriveSubsystem.getSpeedMultiplier() * 100));
    }));
    driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {
      m_DriveSubsystem.decreaseSpeed();
      SmartDashboard.putString("Status/Last Action",
          "Speed: " + String.format("%.0f%%", m_DriveSubsystem.getSpeedMultiplier() * 100));
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
    
    // Test: A button = pure forward drive (1 m/s, no joystick, no field-relative)
    // If robot spins during this, the issue is module config. If straight, issue is joystick/field-relative.
    driverXbox.a()
      .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Status/Last Action", "TEST FORWARD DRIVE")))
      .whileTrue(m_DriveSubsystem.testDriveForwardCommand());
    
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
   * Uses PathPlanner auto chooser if configured, otherwise falls back to manual chooser.
   */
  public Command getAutonomousCommand() {
    // If DriveSubsystem failed to initialize, return do-nothing command
    if (m_DriveSubsystem == null) {
      DriverStation.reportWarning("Cannot run autonomous - DriveSubsystem not initialized", false);
      return Autos.doNothingAuto();
    }
    
    // Get command from auto chooser (PathPlanner or manual)
    Command selectedAuto = autoChooser.getSelected();
    
    if (selectedAuto != null) {
      System.out.println("[RobotContainer] Selected autonomous: " + selectedAuto.getName());
      return selectedAuto;
    }
    
    return Autos.doNothingAuto();
  }

  /**
   * Returns true if the Driver Station reports Red alliance (or defaults to false/Blue if unknown).
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Returns the correct starting pose based on the current alliance color.
   */
  private Pose2d getAllianceStartPose() {
    return isRedAlliance() ? FieldConstants.RED_ALLIANCE_START : FieldConstants.BLUE_ALLIANCE_START;
  }

  /**
   * Called when teleop starts.
   */
  public void teleopInit() {
    if (m_DriveSubsystem == null) {
      DriverStation.reportWarning("DriveSubsystem not available for teleop", false);
      return;
    }
    
    // Reset odometry to the correct alliance start position
    Pose2d startPose = getAllianceStartPose();
    m_DriveSubsystem.resetOdometry(startPose);
    System.out.println("Teleop initialized - odometry reset to " +
        (isRedAlliance() ? "Red" : "Blue") + " Alliance start");
  }  
}
