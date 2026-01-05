package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveDrive swerveDrive;
    public SwerveSubsystem(File directory) {
        try {
            this.swerveDrive = new SwerveParser(directory).createSwerveDrive(5);
        } catch (Exception e) {
            throw new java.lang.RuntimeException(e); // or remove/rename custom RuntimeException class
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }
    private void checkDirectory(File directory) throws IOException {
        if (!new File(directory, "swervedrive.json").exists()) {
            throw new IOException("Missing swervedrive.json in " + directory);
        }
        // check other files similarly
    }
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()),
                3.0
            );
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(scaledInputs.getX(), scaledInputs.getY(), 0.0);
            this.swerveDrive.drive(chassisSpeeds, new Translation2d()); // Adjust Translation2d if necessary
        });
    }
}
