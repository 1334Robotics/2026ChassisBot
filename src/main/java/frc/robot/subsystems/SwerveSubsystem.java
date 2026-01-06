package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory) {
        try {
            this.swerveDrive = new SwerveParser(directory).createSwerveDrive(5);
        } catch (Exception e) {
            throw new java.lang.RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }



    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            // Step 1: Get joystick values and scale them
            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()),
                3.0
            );

            // Step 2: Calculate rotation (with deadband so small wiggles ignored)
            double omega = MathUtil.applyDeadband(headingX.getAsDouble(), 0.05) * 5.0;

            // Step 3: Build the speed command
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                scaledInputs.getX(),  // forward/back
                scaledInputs.getY(),  // left/right
                omega                 // spin
            );

            // Step 4: Drive ONCE
            driveFieldOriented(chassisSpeeds);
        });
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void seedForwards() {
        var pose = swerveDrive.getPose();
        swerveDrive.resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero));
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }
}
