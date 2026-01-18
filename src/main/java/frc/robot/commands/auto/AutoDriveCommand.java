package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Drive to a target pose autonomously.
 * Moves in straight lines between waypoints using field-oriented control.
 */
public class AutoDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Pose2d targetPose;
    private final double maxSpeed;
    private final double timeoutSeconds;
    private double startTime;
    
    private static final double POSITION_TOLERANCE = 0.20; // meters
    private static final double ANGLE_TOLERANCE = 15.0; // degrees
    private static final double KP_LINEAR = 2.0;
    private static final double KP_ANGULAR = 1.5;
    
    public AutoDriveCommand(DriveSubsystem drive, Pose2d targetPose, double maxSpeed, double timeoutSeconds) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        
        Pose2d currentPose = drive.getPose();
        System.out.println("[AutoDriveCommand] Moving from (" + 
            String.format("%.2f, %.2f @ %.0f°", currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()) + 
            ") to (" + String.format("%.2f, %.2f @ %.0f°", targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()) + ")");
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        
        // Calculate error to target in field frame
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);
        
        // Calculate angle error
        double angleError = normalizeAngle(
            targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()
        );
        
        // If close enough to target position, focus on rotation
        if (distance < POSITION_TOLERANCE) {
            if (Math.abs(angleError) < ANGLE_TOLERANCE) {
                drive.stop();
                return;
            }
            
            double omega = KP_ANGULAR * Math.toRadians(angleError);
            omega = clamp(omega, -Math.PI, Math.PI);
            drive.driveFieldOriented(new ChassisSpeeds(0, 0, omega));
            return;
        }
        
        // Calculate speed toward target with proportional control
        double speed = Math.min(KP_LINEAR * distance, maxSpeed);
        speed = Math.max(0.3, speed); // Minimum speed
        
        // Normalize direction vector and apply speed
        double vx = (dx / distance) * speed;
        double vy = (dy / distance) * speed;
        
        // Also correct heading while moving
        double omega = KP_ANGULAR * Math.toRadians(angleError);
        omega = clamp(omega, -Math.PI / 2, Math.PI / 2); // Limit rotation speed
        
        // Send velocities to drive (field-oriented)
        drive.driveFieldOriented(new ChassisSpeeds(vx, vy, omega));
    }
    
    @Override
    public boolean isFinished() {
        Pose2d currentPose = drive.getPose();
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);
        
        double angleError = Math.abs(normalizeAngle(
            targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()
        ));
        
        boolean atTarget = distance < POSITION_TOLERANCE && angleError < ANGLE_TOLERANCE;
        boolean timedOut = (Timer.getFPGATimestamp() - startTime) > timeoutSeconds;
        
        if (atTarget) {
            System.out.println("[AutoDriveCommand] [OK] Reached target");
        }
        if (timedOut) {
            System.out.println("[AutoDriveCommand] [WARN] Timed out - distance: " + 
                String.format("%.2f", distance) + "m, angle error: " + 
                String.format("%.1f", angleError) + "°");
        }
        
        return atTarget || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        if (interrupted) {
            System.out.println("[AutoDriveCommand] [WARN] Interrupted");
        }
    }
    
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
