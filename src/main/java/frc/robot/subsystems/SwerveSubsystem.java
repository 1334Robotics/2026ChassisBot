// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {

File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive  swerveDrive; 

  public SwerveSubsystem() {
        try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
                                                                                       new Pose2d( new Translation2d(Meter.of(1),
                                                                                                                     Meter.of(4)),
                                                                                        Rotation2d.fromDegrees(0)
                                                                                      );
                                                                  
      
                                                                                      swerveDrive.resetOdometry(new Pose2d(new Translation2d(5.0, 4.0), Rotation2d.fromDegrees(0)));

    }    catch (Exception e)
    {
      throw new RuntimeException(e);}
    }


  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }


public void driveFieldOriented (ChassisSpeeds velocity){
  swerveDrive.driveFieldOriented(velocity);
}
public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
  return run(() -> {
    swerveDrive.driveFieldOriented(velocity.get());
  });
}
}