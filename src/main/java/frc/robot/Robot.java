// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putString("Robot/Mode", "Initializing");
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // Update match time on dashboard
    SmartDashboard.putNumber("Robot/Match Time", edu.wpi.first.wpilibj.Timer.getMatchTime());
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
    SmartDashboard.putString("Status/Auto Running", "Starting...");
    
    // Get a fresh autonomous command each time
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      SmartDashboard.putString("Status/Auto Running", m_autonomousCommand.getName());
    } else {
      SmartDashboard.putString("Status/Auto Running", "None Selected");
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
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
