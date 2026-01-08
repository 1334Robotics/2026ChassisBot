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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SwerveDrive swerveDrive;
  private final Field2d field = new Field2d();
  private static final double FIELD_WIDTH_M = 8.23;
  private static final double FIELD_LENGTH_M = 16.46;
  private static final double FIELD_PADDING_M = 0.05;

  public SwerveSubsystem(File directory) {
    try
    {
      this.swerveDrive = new SwerveParser(directory).createSwerveDrive(5);
      this.swerveDrive.resetOdometry(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    // Set telemetry verbosity to high for detailed debugging
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // Add the field to the SmartDashboard for visualization
    SmartDashboard.putData("Field", field);
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
           Translation2d scaledInputs = SwerveMath.scaleTranslation(
           new Translation2d(translationX.getAsDouble(),translationY.getAsDouble()),
          3.0
          );
  
        speeds.vxMetersPerSecond = scaledInputs.getX();
        speeds.vyMetersPerSecond = scaledInputs.getY();
        speeds.omegaRadiansPerSecond = MathUtil.applyDeadband(headingX.getAsDouble(), 0.05) * 5;
        // Make the robot move
        driveFieldOriented(speeds);
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
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Drives the robot in a field-oriented manner using the given chassis speeds.
   *
   * @param speeds The desired chassis speeds.
   */
  public void driveFieldOriented(ChassisSpeeds speeds) {
    if (swerveDrive == null) {
        throw new RuntimeException("swerveDrive is not initialized!");
    }
    swerveDrive.driveFieldOriented(speeds);
  }

  /**
   * Clamps the robot's pose to ensure it stays within defined field boundaries.
   *
   * @param pose The current pose of the robot.
   */
  private void clampPoseIfOutOfBounds(Pose2d pose) {
    double x = MathUtil.clamp(pose.getX(), FIELD_PADDING_M, FIELD_LENGTH_M - FIELD_PADDING_M);
    double y = MathUtil.clamp(pose.getY(), FIELD_PADDING_M, FIELD_WIDTH_M - FIELD_PADDING_M);
    if (x != pose.getX() || y != pose.getY()) {
        swerveDrive.resetOdometry(new Pose2d(x, y, pose.getRotation()));
        SmartDashboard.putString("Swerve/Clamp", "Clamped pose to field bounds");
    };
  }

  @Override
  public void periodic() {
        Pose2d pose = swerveDrive.getPose();
        field.setRobotPose(pose);
        SmartDashboard.putNumber("Swerve/Pose X", pose.getX());
        SmartDashboard.putNumber("Swerve/Pose Y", pose.getY());
        SmartDashboard.putNumber("Swerve/Heading", pose.getRotation().getDegrees());
        clampPoseIfOutOfBounds(pose);
  
}
}
