// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private boolean robotInitialized = false;
  private static final long INIT_TIMEOUT_MS = 5000; // 5 second timeout

  @Override
  public void robotInit() {
    long startTime = System.currentTimeMillis();
    try {
      m_robotContainer = new RobotContainer();
      robotInitialized = true;
      SmartDashboard.putBoolean("Robot/Initialized", true);
      SmartDashboard.putString("Robot/Mode", "Initialized");
      SmartDashboard.putNumber("Robot/Init Time (ms)", System.currentTimeMillis() - startTime);

      // Simulation info
      if (RobotBase.isSimulation()) {
        SmartDashboard.putBoolean("Simulation/Running", true);
        SmartDashboard.putString("Simulation/Info", "Use Field2d widget to view robot");
      }
    } catch (Exception e) {
      DriverStation.reportError("Robot initialization failed: " + e.getMessage(), e.getStackTrace());
      SmartDashboard.putBoolean("Robot/Initialized", false);
      SmartDashboard.putString("Robot/Error", e.getMessage());
      robotInitialized = false;
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    try {
      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
      // commands, running already-scheduled commands, removing finished or interrupted commands,
      // and running subsystem periodic() methods.  This must be called from the robot's periodic
      // block in order for anything in the Command-based framework to work.
      CommandScheduler.getInstance().run();

      // Update match time on dashboard
      SmartDashboard.putNumber("Robot/Match Time", Timer.getMatchTime());
      SmartDashboard.putNumber("Robot/FPGA Time", Timer.getFPGATimestamp());
    } catch (Exception e) {
      DriverStation.reportError("robotPeriodic error: " + e.getMessage(), false);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SmartDashboard.putString("Robot/Mode", "DISABLED");
    SmartDashboard.putBoolean("Robot/Enabled", false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putString("Robot/Mode", "AUTONOMOUS");
    SmartDashboard.putBoolean("Robot/Enabled", true);

    if (!robotInitialized || m_robotContainer == null) {
      DriverStation.reportError("Cannot run autonomous - robot not initialized", false);
      return;
    }

    try {
      // Small delay to ensure systems are ready
      Thread.sleep(100);
      
      // Get a fresh autonomous command each time
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      // Schedule the autonomous command
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
        SmartDashboard.putString("Status/Auto Running", m_autonomousCommand.getName());
        System.out.println("[Robot] Autonomous started: " + m_autonomousCommand.getName());
      } else {
        SmartDashboard.putString("Status/Auto Running", "None (Error: null command)");
      }
    } catch (Exception e) {
      DriverStation.reportError("autonomousInit error: " + e.getMessage(), e.getStackTrace());
      SmartDashboard.putString("Status/Auto Running", "Error: " + e.getMessage());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Robot/Mode", "TELEOP");
    SmartDashboard.putBoolean("Robot/Enabled", true);
    SmartDashboard.putString("Status/Auto Running", "Cancelled");

    // Cancel autonomous command when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    // Initialize teleop
    if (robotInitialized && m_robotContainer != null) {
      try {
        m_robotContainer.teleopInit();
      } catch (Exception e) {
        DriverStation.reportError("teleopInit error: " + e.getMessage(), false);
      }
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    SmartDashboard.putString("Robot/Mode", "TEST");
    SmartDashboard.putBoolean("Robot/Enabled", true);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SmartDashboard.putBoolean("Robot/Simulation", true);
    SmartDashboard.putString("Robot/Mode", "Simulation Init");
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
