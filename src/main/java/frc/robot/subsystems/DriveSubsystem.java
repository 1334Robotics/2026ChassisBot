package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;

public class DriveSubsystem {
    public DriveSubsystem() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        SwerveDrive  swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.SwerveConstants.MAXIMUM_SPEED);

    }
}