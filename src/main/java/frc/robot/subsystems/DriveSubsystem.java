package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase{
    public DriveSubsystem() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try{
            SwerveDrive  swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.SwerveConstants.MAXIMUM_SPEED);
        }
        catch(IOException e) { throw new RuntimeException(e); }
    }
}