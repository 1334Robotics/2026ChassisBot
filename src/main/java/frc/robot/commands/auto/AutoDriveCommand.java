package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Drive to a target pose autonomously with timeout protection.
 */
public class AutoDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Pose2d targetPose;
    private final double maxSpeed;
    private final double timeoutSeconds;
    private double startTime;
    
    private static final double POSITION_TOLERANCE = 0.1; // meters
    private static final double ANGLE_TOLERANCE = 5.0; // degrees
    private static final double KP_LINEAR = 2.0;
    private static final double KP_ANGULAR = 3.0;
    
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
        drive.zeroGyro();
        
        Pose2d currentPose = drive.getPose();
        System.out.println("[AutoDriveCommand] Starting from (" + 
            String.format("%.2f, %.2f", currentPose.getX(), currentPose.getY()) + 
            ") heading to (" + 
            String.format("%.2f, %.2f", targetPose.getX(), targetPose.getY()) + ")");
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        
        // Calculate position error
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);
        
        // If very close to target, stop moving to prevent jitter
        if (distance < 0.05) {
            drive.stop();
            System.out.println("[AutoDriveCommand] Reached target position");
            return;
        }
        
        if (distance < 0.1) {
            // Already at target position, focus on rotation only
            double angleError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
            angleError = normalizeAngle(angleError);
            
            if (Math.abs(angleError) < ANGLE_TOLERANCE) {
                drive.stop();
                return;
            }
            
            double angularSpeed = Math.min(KP_ANGULAR * Math.toRadians(angleError), 2.0 * Math.PI);
            drive.driveFieldOriented(new ChassisSpeeds(0, 0, angularSpeed));
            return;
        }
        
        // Calculate angle to target
        double targetHeading = Math.atan2(dy, dx);
        double currentHeading = currentPose.getRotation().getRadians();
        double headingError = targetHeading - currentHeading;
        
        // Normalize heading error to [-π, π]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        
        // PID control for speed toward target
        double linearSpeed = Math.min(KP_LINEAR * distance, maxSpeed);
        
        // Drive directly toward target in field-oriented mode
        double vx = linearSpeed * Math.cos(targetHeading);
        double vy = linearSpeed * Math.sin(targetHeading);
        
        // Also correct heading if needed
        double angleError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
        angleError = normalizeAngle(angleError);
        double angularSpeed = Math.min(KP_ANGULAR * Math.toRadians(angleError), 2.0 * Math.PI);
        
        drive.driveFieldOriented(new ChassisSpeeds(vx, vy, angularSpeed));
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
            System.out.println("[AutoDriveCommand] Reached target successfully");
        }
        if (timedOut) {
            System.out.println("[AutoDriveCommand] Command timed out at (" + 
                String.format("%.2f, %.2f", currentPose.getX(), currentPose.getY()) + ")");
        }
        
        return atTarget || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        if (interrupted) {
            System.out.println("[AutoDriveCommand] Interrupted");
        }
    }
    
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
