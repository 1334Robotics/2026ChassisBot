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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final Field2d field = new Field2d();
    private static final double FIELD_WIDTH_M = 8.23;
    private static final double FIELD_LENGTH_M = 16.46;
    private static final double FIELD_PADDING_M = 0.05;

    public SwerveSubsystem(File directory) {
        try {
            this.swerveDrive = new SwerveParser(directory).createSwerveDrive(5);
            // Ensure robot spawns on the field in simulation (meters).
            // Pick coordinates inside the 27ft x 54ft field (≈ 8.23m x 16.46m) e.g. (1,1)
            this.swerveDrive.resetOdometry(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new java.lang.RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SmartDashboard.putData("Field", field);
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

    private void clampPoseIfOutOfBounds(Pose2d pose) {
        double x = MathUtil.clamp(pose.getX(), FIELD_PADDING_M, FIELD_LENGTH_M - FIELD_PADDING_M);
        double y = MathUtil.clamp(pose.getY(), FIELD_PADDING_M, FIELD_WIDTH_M - FIELD_PADDING_M);
        if (x != pose.getX() || y != pose.getY()) {
            swerveDrive.resetOdometry(new Pose2d(x, y, pose.getRotation()));
            SmartDashboard.putString("Swerve/Clamp", "Clamped pose to field bounds");
        }
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
