package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends SubsystemBase
{
    SwerveDrive swerveDrive;
    private final Field2d field2d;
    
    // Maximum speeds
    private static final double MAX_SPEED_MPS = 3.0; // meters per second
    private static final double MAX_ANGULAR_VELOCITY = Math.PI; // radians per second (180 deg/s)
    
    public DriveSubsystem(File directory) {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "SWERVE");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED_MPS);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        
        // Initialize Field2d for visualization
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);
    }
    
    /**
     * Command to drive the robot using joystick inputs.
     *
     * @param translationX Forward/backward speed supplier (-1.0 to 1.0).
     * @param translationY Left/right speed supplier (-1.0 to 1.0).
     * @param rotationX    Rotation speed supplier (-1.0 to 1.0).
     * @param headingY     Not used.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotationX,
                                DoubleSupplier headingY)
    {
        return run(() -> {
            // Get joystick inputs
            double xInput = translationX.getAsDouble();
            double yInput = translationY.getAsDouble();
            double rotInput = rotationX.getAsDouble();
            
            // Convert to velocities (m/s and rad/s)
            double xVelocity = xInput * MAX_SPEED_MPS;
            double yVelocity = yInput * MAX_SPEED_MPS;
            double rotVelocity = rotInput * MAX_ANGULAR_VELOCITY;
            
            // Create chassis speeds and drive
            ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, rotVelocity);
            swerveDrive.driveFieldOriented(speeds);
        });
    }
    
    /**
     * Stop the robot completely.
     */
    public void stop() {
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Lock the swerve modules to prevent movement.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    public void seedForwards() {
        var pose = swerveDrive.getPose();
        swerveDrive.resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero));
    }
    
    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Get the current pose of the robot.
     *
     * @return Current robot pose.
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Reset odometry to a specific pose.
     *
     * @param pose The pose to reset to.
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Update robot pose with vision measurements.
     *
     * @param visionPose The pose from vision system.
     * @param timestamp The timestamp of the vision measurement.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        swerveDrive.addVisionMeasurement(visionPose, timestamp);
    }

    /**
     * Get the Field2d object for adding trajectories.
     *
     * @return The Field2d object.
     */
    public Field2d getField2d() {
        return field2d;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Adjusted Front Left", swerveDrive.getModules()[0].getAbsolutePosition());
        SmartDashboard.putNumber("Adjusted Front Right", swerveDrive.getModules()[1].getAbsolutePosition());
        SmartDashboard.putNumber("Adjusted Back Left", swerveDrive.getModules()[2].getAbsolutePosition());
        SmartDashboard.putNumber("Adjusted Back Right", swerveDrive.getModules()[3].getAbsolutePosition());
        SmartDashboard.putNumber("heading", swerveDrive.getOdometryHeading().getDegrees());
        
        // Update Field2d with current robot pose
        field2d.setRobotPose(swerveDrive.getPose());
        
        // Additional pose data to dashboard
        Pose2d pose = swerveDrive.getPose();
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
    }
}