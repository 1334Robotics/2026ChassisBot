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
    
    // Track current speeds for dashboard
    private double currentXSpeed = 0.0;
    private double currentYSpeed = 0.0;
    private double currentRotSpeed = 0.0;
    
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
        
        // Put initial values on dashboard
        SmartDashboard.putNumber("Drive/Max Speed (m/s)", MAX_SPEED_MPS);
        SmartDashboard.putNumber("Drive/Max Angular Velocity (rad/s)", MAX_ANGULAR_VELOCITY);
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
            currentXSpeed = xInput * MAX_SPEED_MPS;
            currentYSpeed = yInput * MAX_SPEED_MPS;
            currentRotSpeed = rotInput * MAX_ANGULAR_VELOCITY;
            
            // Create chassis speeds and drive
            ChassisSpeeds speeds = new ChassisSpeeds(currentXSpeed, currentYSpeed, currentRotSpeed);
            swerveDrive.driveFieldOriented(speeds);
        });
    }
    
    /**
     * Stop the robot completely.
     */
    public void stop() {
        currentXSpeed = 0.0;
        currentYSpeed = 0.0;
        currentRotSpeed = 0.0;
        swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("Drive/Status", "Stopped");
    }

    /**
     * Lock the swerve modules to prevent movement.
     */
    public void lock() {
        swerveDrive.lockPose();
        SmartDashboard.putBoolean("Drive/Locked", true);
    }

    public void seedForwards() {
        var pose = swerveDrive.getPose();
        swerveDrive.resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero));
        SmartDashboard.putBoolean("Drive/Heading Reset", true);
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
        SmartDashboard.putString("Drive/Last Reset Pose", 
            String.format("(%.2f, %.2f) @ %.1f°", pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
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
        // Update Field2d with current robot pose
        Pose2d pose = swerveDrive.getPose();
        field2d.setRobotPose(pose);
        
        // ==================== POSE DATA ====================
        SmartDashboard.putNumber("Drive/Pose X (m)", pose.getX());
        SmartDashboard.putNumber("Drive/Pose Y (m)", pose.getY());
        SmartDashboard.putNumber("Drive/Pose Rotation (deg)", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive/Heading (deg)", swerveDrive.getOdometryHeading().getDegrees());
        
        // ==================== VELOCITY DATA ====================
        SmartDashboard.putNumber("Drive/X Velocity (m/s)", currentXSpeed);
        SmartDashboard.putNumber("Drive/Y Velocity (m/s)", currentYSpeed);
        SmartDashboard.putNumber("Drive/Rotation Velocity (rad/s)", currentRotSpeed);
        SmartDashboard.putNumber("Drive/Total Speed (m/s)", 
            Math.sqrt(currentXSpeed * currentXSpeed + currentYSpeed * currentYSpeed));
        
        // ==================== MODULE DATA ====================
        SmartDashboard.putNumber("Swerve/FL Angle", swerveDrive.getModules()[0].getAbsolutePosition());
        SmartDashboard.putNumber("Swerve/FR Angle", swerveDrive.getModules()[1].getAbsolutePosition());
        SmartDashboard.putNumber("Swerve/BL Angle", swerveDrive.getModules()[2].getAbsolutePosition());
        SmartDashboard.putNumber("Swerve/BR Angle", swerveDrive.getModules()[3].getAbsolutePosition());
        
        // ==================== STATUS ====================
        SmartDashboard.putBoolean("Drive/Locked", false);
        SmartDashboard.putBoolean("Drive/Heading Reset", false);
    }
}