// Copyright (c) FIRST and other WPILib contributors.
// Open Source Softshare; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;


import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SwerveDrive swerveDrive;
  public SwerveSubsystem(File directory) {
    try
    {
      this.swerveDrive = new SwerveParser(directory).createSwerveDrive(5);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
  DoubleSupplier headingY) {
          ChassisSpeeds speeds = new ChassisSpeeds();
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */ 
           Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                   translationY.getAsDouble()), 3.0);
  

        speeds.vxMetersPerSecond = scaledInputs.getX();
        speeds.vyMetersPerSecond = scaledInputs.getY();
        speeds.omegaRadiansPerSecond = MathUtil.applyDeadband(headingX.getAsDouble(), 0.05) * 5;
        // Make the robot move
        driveFieldOriented(speeds);
        //driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        //                                                                 headingX.getAsDouble(),
        //                                                                 headingY.getAsDouble(),
        //                                                                 swerveDrive.getOdometryHeading().getRadians(),
        //                                                                 swerveDrive.getMaximumChassisVelocity()));
      });
    };
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public void seedForwards() {
    var pose = swerveDrive.getPose();
    swerveDrive.resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero));
  }
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /**
   * Drives the robot in a field-oriented manner using the given chassis speeds.
   *
   * @param speeds The desired chassis speeds.
   */
  public void driveFieldOriented(ChassisSpeeds speeds) {
    swerveDrive.driveFieldOriented(speeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Adjusted Front Left", swerveDrive.getModules()[0].getAbsolutePosition());
    SmartDashboard.putNumber("Adjusted Front Right", swerveDrive.getModules()[1].getAbsolutePosition());
    SmartDashboard.putNumber("Adjusted Back Left", swerveDrive.getModules()[2].getAbsolutePosition());
    SmartDashboard.putNumber("Adjusted Back Right", swerveDrive.getModules()[3].getAbsolutePosition());
    SmartDashboard.putNumber("heading", swerveDrive.getOdometryHeading().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
