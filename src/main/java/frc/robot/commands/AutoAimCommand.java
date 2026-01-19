// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends Command {
  private final VisionSubsystem vision;
  private final SwerveSubsystem drive;

  private static final double ANGLE_TOLERANCE = 1.0;
  private static final double MAX_ROTATION = 0.3;

  public AutoAimCommand(VisionSubsystem vision, SwerveSubsystem drive) {
    this.vision = vision;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!vision.hasTarget()) {
        return;
    }

    double error = vision.getX();

    double rotationSpeed = error * 0.02;

    if (rotationSpeed > MAX_ROTATION) rotationSpeed = MAX_ROTATION;
    if (rotationSpeed < -MAX_ROTATION) rotationSpeed = -MAX_ROTATION;

    drive.driveFieldOriented(new ChassisSpeeds(0, 0, rotationSpeed));
  }

  @Override
  public void end(boolean interrupted) {
      drive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
      if (!vision.hasTarget()) return false;

      double error = vision.getX();
      return Math.abs(error) < ANGLE_TOLERANCE; 
  }
}