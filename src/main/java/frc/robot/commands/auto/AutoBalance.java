package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Auto-balance on a charge station (requires gyro).
 * Drives onto the charge station and balances by monitoring pitch angle.
 */
public class AutoBalance extends Command {
    private final DriveSubsystem drive;
    private final PIDController balancePID;
    
    // Charge station constants
    private static final double DRIVE_SPEED = 2.0; // m/s
    private static final double TILT_THRESHOLD = 12.0; // degrees - when we detect the ramp
    private static final double BALANCED_THRESHOLD = 3.0; // degrees - tolerance for balanced state
    private static final double TIMEOUT_SECONDS = 10.0;
    private static final double BALANCE_TIME_REQUIRED = 1.0; // seconds to be balanced
    
    // PID constants for balancing
    private static final double KP = 0.05;
    private static final double KI = 0.01;
    private static final double KD = 0.02;
    
    private double startTime;
    private double balancedStartTime;
    private boolean onRamp = false;
    
    public AutoBalance(DriveSubsystem drive) {
        this.drive = drive;
        this.balancePID = new PIDController(KP, KI, KD);
        this.balancePID.setTolerance(BALANCED_THRESHOLD);
        
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        balancedStartTime = 0;
        onRamp = false;
        
        drive.zeroGyro();
        System.out.println("AutoBalance: Starting charge station balance routine");
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Get current pitch from gyro
        double currentPitch = drive.getPose().getRotation().getDegrees();
        
        if (!onRamp) {
            // Phase 1: Drive forward onto ramp
            if (Math.abs(currentPitch) < TILT_THRESHOLD) {
                // Not on ramp yet, keep driving forward
                drive.driveFieldOriented(new ChassisSpeeds(DRIVE_SPEED, 0, 0));
            } else {
                // Detected ramp tilt
                onRamp = true;
                balancedStartTime = currentTime;
                System.out.println("AutoBalance: Detected ramp, entering balance mode. Pitch: " + currentPitch);
            }
        } else {
            // Phase 2: Balance on ramp using PID
            double pidOutput = balancePID.calculate(currentPitch, 0.0);
            
            // Clamp output to reasonable drive speed
            pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
            
            // Drive to correct tilt
            drive.driveFieldOriented(new ChassisSpeeds(pidOutput * DRIVE_SPEED, 0, 0));
            
            // Check if balanced
            if (Math.abs(currentPitch) <= BALANCED_THRESHOLD) {
                if (balancedStartTime == 0) {
                    balancedStartTime = currentTime;
                }
            } else {
                // Reset balance timer if tilt increases
                balancedStartTime = 0;
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Timeout safety
        if (currentTime - startTime > TIMEOUT_SECONDS) {
            System.out.println("AutoBalance: Timeout reached");
            return true;
        }
        
        // Check if balanced for required duration
        if (onRamp && balancedStartTime > 0) {
            double balanceTime = currentTime - balancedStartTime;
            if (balanceTime >= BALANCE_TIME_REQUIRED) {
                System.out.println("AutoBalance: Successfully balanced for " + balanceTime + " seconds");
                return true;
            }
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        drive.lock();
        
        if (interrupted) {
            System.out.println("AutoBalance: Interrupted");
        } else {
            System.out.println("AutoBalance: Complete");
        }
    }
}
