package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase{
    private SwerveDrive swerveDrive;

    public DriveSubsystem() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.SwerveConstants.MAXIMUM_SPEED);
            if (swerveDrive == null) {
              System.out.println("SwerveDrive failed to initialize!");
          }
        }
        catch(IOException e) { throw new RuntimeException(e); }
    }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
    });
  }
}