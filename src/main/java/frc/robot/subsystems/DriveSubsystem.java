package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends SubsystemBase
{
    private SwerveDrive swerveDrive;
    private final Field2d field2d;
    private boolean initialized = false;
    private boolean hasZeroed = false;
    private long lastErrorTime = 0;
    private static final long ERROR_THROTTLE_MS = 500; // Don't spam errors
    
    // Track current speeds for dashboard
    private double currentXSpeed = 0.0;
    private double currentYSpeed = 0.0;
    private double currentRotSpeed = 0.0;
    
    // Simulation pose tracking (when hardware isn't available)
    private Pose2d simPose = new Pose2d();
    private double lastUpdateTime = 0;
    
    // Startup delay to allow modules to initialize
    private double startupTime = 0;
    private static final double STARTUP_DELAY_SECONDS = 0.5;
    private static final double MAX_DT = 0.1;
    private static final double DEADZONE_THRESHOLD = 0.05;
    private static final double MAX_SPEED_LIMIT = 5.0; // Safety limit
    
    public DriveSubsystem(File directory) {
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);
        
        initializeSwerve();
        
        lastUpdateTime = Timer.getFPGATimestamp();
        startupTime = Timer.getFPGATimestamp();
    }
    
    /**
     * Initialize swerve drive with detailed error handling and logging.
     */
    private void initializeSwerve() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "SWERVE");
            
            if (!swerveJsonDirectory.exists()) {
                logError("SWERVE directory not found: " + swerveJsonDirectory.getAbsolutePath());
                return;
            }
            
            logDebug("Creating SwerveDrive with max speed: " + DriveConstants.MAX_SPEED_MPS + " m/s");
            logDebug("Running in " + (RobotBase.isSimulation() ? "SIMULATION" : "REAL") + " mode");
            
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.MAX_SPEED_MPS);
            
            verifyModules();
            initialized = true;
            
            zeroGyro();
            synchronizeModuleEncoders();
            SmartDashboard.putBoolean("Drive/Initialized", true);
            logDebug("Swerve drive initialized successfully");
            
        } catch (Exception e) {
            logError("FATAL ERROR during swerve initialization: " + e.getMessage());
            e.printStackTrace();
            SmartDashboard.putString("Drive/Error", "Init failed: " + e.getMessage());
            SmartDashboard.putBoolean("Drive/Initialized", false);
        }
        
        SmartDashboard.putNumber("Drive/Max Speed (m/s)", DriveConstants.MAX_SPEED_MPS);
        SmartDashboard.putNumber("Drive/Max Angular Velocity (rad/s)", DriveConstants.MAX_ANGULAR_VELOCITY);
    }
    
    /**
     * Verify all modules loaded correctly.
     */
    private void verifyModules() {
        var modules = swerveDrive.getModules();
        logDebug("Loaded " + modules.length + " swerve modules");
        SmartDashboard.putNumber("Drive/Module Count", modules.length);
        
        int validModules = 0;
        for (int i = 0; i < modules.length; i++) {
            boolean isValid = modules[i] != null;
            if (isValid) validModules++;
            logDebug("Module " + i + ": " + (isValid ? "OK" : "NULL"));
            SmartDashboard.putBoolean("Drive/Module " + i + " OK", isValid);
        }
        
        if (validModules != 4) {
            logError("Only " + validModules + "/4 modules loaded successfully");
        }
    }
    
    /**
     * Throttled error logging to avoid spam.
     */
    private void logError(String message) {
        long now = System.currentTimeMillis();
        if (now - lastErrorTime > ERROR_THROTTLE_MS) {
            System.out.println("[DriveSubsystem ERROR] " + message);
            SmartDashboard.putString("Drive/Error", message);
            DriverStation.reportError(message, false);
            lastErrorTime = now;
        }
    }
    
    /**
     * Log debug message to console.
     */
    private void logDebug(String message) {
        System.out.println("[DriveSubsystem] " + message);
    }
    
    /**
     * Zero the gyroscope heading.
     */
    public void zeroGyro() {
        if (!isInitialized()) return;
        
        try {
            swerveDrive.zeroGyro();
            SmartDashboard.putBoolean("Drive/Gyro Zeroed", true);
            logDebug("Gyro zeroed");
        } catch (Exception e) {
            logError("zeroGyro error: " + e.getMessage());
        }
    }
    
    /**
     * Synchronize module encoders to absolute encoders.
     */
    public void synchronizeModuleEncoders() {
        if (!isInitialized()) return;
        
        try {
            swerveDrive.synchronizeModuleEncoders();
            hasZeroed = true;
            SmartDashboard.putBoolean("Drive/Modules Synced", true);
            logDebug("Module encoders synchronized");
        } catch (Exception e) {
            DriverStation.reportWarning("synchronizeModuleEncoders error: " + e.getMessage(), false);
        }
    }
    
    /**
     * Check if the drive subsystem is properly initialized.
     */
    public boolean isInitialized() {
        return initialized && swerveDrive != null;
    }
    
    /**
     * Check if we're past the startup delay.
     */
    private boolean isReadyToDrive() {
        return (Timer.getFPGATimestamp() - startupTime) > STARTUP_DELAY_SECONDS;
    }
    
    /**
     * Clamp a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Apply deadzone to joystick input.
     */
    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE_THRESHOLD ? 0.0 : value;
    }
    
    /**
     * Clamp speed with safety limits.
     */
    private ChassisSpeeds limitSpeeds(ChassisSpeeds speeds) {
        double linearMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularMagnitude = Math.abs(speeds.omegaRadiansPerSecond);
        
        if (linearMagnitude == 0 && angularMagnitude == 0) {
            return speeds;
        }
        
        double linearLimit = Math.min(1.0, MAX_SPEED_LIMIT / Math.max(linearMagnitude, 0.01));
        double angularLimit = Math.min(1.0, DriveConstants.MAX_ANGULAR_VELOCITY / Math.max(angularMagnitude, 0.01));
        
        double limitFactor = Math.min(linearLimit, angularLimit);
        
        return new ChassisSpeeds(
            speeds.vxMetersPerSecond * limitFactor,
            speeds.vyMetersPerSecond * limitFactor,
            speeds.omegaRadiansPerSecond * limitFactor
        );
    }
    
    /**
     * Command to drive the robot using joystick inputs.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                                DoubleSupplier rotationX) {
        return run(() -> {
            if (!isReadyToDrive()) {
                stop();
                return;
            }
            
            try {
                double xInput = applyDeadzone(clamp(translationX.getAsDouble(), -1.0, 1.0));
                double yInput = applyDeadzone(clamp(translationY.getAsDouble(), -1.0, 1.0));
                double rotInput = applyDeadzone(clamp(rotationX.getAsDouble(), -1.0, 1.0));
                
                currentXSpeed = xInput * DriveConstants.MAX_SPEED_MPS;
                currentYSpeed = yInput * DriveConstants.MAX_SPEED_MPS;
                currentRotSpeed = rotInput * DriveConstants.MAX_ANGULAR_VELOCITY;
                
                ChassisSpeeds speeds = new ChassisSpeeds(currentXSpeed, currentYSpeed, currentRotSpeed);
                driveFieldOrientedSafe(limitSpeeds(speeds));
                
            } catch (Exception e) {
                logError("Drive command error: " + e.getMessage());
                stop();
            }
        }).withName("DriveCommand");
    }
    
    /**
     * Get a command to zero the gyro and sync modules.
     */
    public Command zeroCommand() {
        return Commands.runOnce(() -> {
            zeroGyro();
            synchronizeModuleEncoders();
        }).withName("ZeroDrive");
    }
    
    /**
     * Safely drive field oriented, handling errors gracefully.
     */
    private void driveFieldOrientedSafe(ChassisSpeeds speeds) {
        if (isInitialized()) {
            try {
                swerveDrive.driveFieldOriented(speeds);
            } catch (Exception e) {
                logError("driveFieldOriented error: " + e.getMessage());
            }
        } else if (RobotBase.isSimulation()) {
            updateSimulatedPose(speeds);
        }
    }
    
    /**
     * Update simulated pose when hardware isn't available.
     */
    private void updateSimulatedPose(ChassisSpeeds speeds) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = clamp(currentTime - lastUpdateTime, 0.0, MAX_DT);
        lastUpdateTime = currentTime;
        
        if (dt < 0.001) return; // Skip negligible time deltas
        
        double dx = speeds.vxMetersPerSecond * dt;
        double dy = speeds.vyMetersPerSecond * dt;
        double dTheta = speeds.omegaRadiansPerSecond * dt;
        
        Pose2d newPose = new Pose2d(
            simPose.getX() + dx,
            simPose.getY() + dy,
            simPose.getRotation().plus(new Rotation2d(dTheta))
        );
        
        simPose = newPose;
    }
    
    /**
     * Stop the robot completely.
     */
    public void stop() {
        currentXSpeed = 0.0;
        currentYSpeed = 0.0;
        currentRotSpeed = 0.0;
        
        if (isInitialized()) {
            try {
                swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            } catch (Exception e) {
                logError("Stop error: " + e.getMessage());
            }
        }
        SmartDashboard.putString("Drive/Status", "Stopped");
    }

    /**
     * Lock the swerve modules to prevent movement.
     */
    public void lock() {
        if (isInitialized()) {
            try {
                swerveDrive.lockPose();
                SmartDashboard.putBoolean("Drive/Locked", true);
            } catch (Exception e) {
                logError("Lock error: " + e.getMessage());
            }
        }
    }

    /**
     * Reset the heading to forward.
     */
    public void seedForwards() {
        try {
            zeroGyro();
            Pose2d pose = getPose();
            resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero));
            SmartDashboard.putBoolean("Drive/Heading Reset", true);
        } catch (Exception e) {
            logError("seedForwards error: " + e.getMessage());
        }
    }
    
    /**
     * Drive the robot given a chassis field oriented velocity.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        if (velocity == null) {
            velocity = new ChassisSpeeds(0, 0, 0);
        }
        
        currentXSpeed = velocity.vxMetersPerSecond;
        currentYSpeed = velocity.vyMetersPerSecond;
        currentRotSpeed = velocity.omegaRadiansPerSecond;
        
        driveFieldOrientedSafe(limitSpeeds(velocity));
    }

    /**
     * Get the current pose of the robot.
     */
    public Pose2d getPose() {
        if (isInitialized()) {
            try {
                return swerveDrive.getPose();
            } catch (Exception e) {
                logError("getPose error: " + e.getMessage());
            }
        }
        return simPose;
    }

    /**
     * Reset odometry to a specific pose.
     */
    public void resetOdometry(Pose2d pose) {
        if (pose == null) {
            pose = new Pose2d();
        }
        
        simPose = pose;
        
        if (isInitialized()) {
            try {
                swerveDrive.resetOdometry(pose);
            } catch (Exception e) {
                logError("resetOdometry error: " + e.getMessage());
            }
        }
        
        SmartDashboard.putString("Drive/Last Reset Pose", 
            String.format("(%.2f, %.2f) @ %.1f°", pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
    }

    /**
     * Update robot pose with vision measurements.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        if (visionPose == null || !isInitialized()) {
            return;
        }
        
        try {
            swerveDrive.addVisionMeasurement(visionPose, timestamp);
        } catch (Exception e) {
            logError("addVisionMeasurement error: " + e.getMessage());
        }
    }

    /**
     * Get the Field2d object for adding trajectories.
     */
    public Field2d getField2d() {
        return field2d;
    }

    /**
     * Get the maximum speed in meters per second.
     */
    public double getMaxSpeed() {
        return DriveConstants.MAX_SPEED_MPS;
    }

    /**
     * Print current absolute encoder positions for calibration.
     */
    public void printEncoderOffsets() {
        if (!isInitialized()) {
            System.out.println("Drive not initialized - cannot read encoders");
            return;
        }
        
        String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
        System.out.println("========== ENCODER OFFSETS ==========");
        
        try {
            var modules = swerveDrive.getModules();
            for (int i = 0; i < modules.length && i < moduleNames.length; i++) {
                if (modules[i] != null) {
                    double angle = modules[i].getAbsolutePosition();
                    System.out.println(moduleNames[i] + ": " + angle + " degrees");
                    SmartDashboard.putNumber("Calibration/" + moduleNames[i] + " Offset", angle);
                }
            }
        } catch (Exception e) {
            logError("Error reading encoders: " + e.getMessage());
        }
        System.out.println("======================================");
    }

    /**
     * Diagnose gyro issues and display info.
     */
    public void diagnoseGyro() {
        if (!isInitialized()) {
            System.out.println("Drive not initialized");
            return;
        }
        
        try {
            var heading = swerveDrive.getOdometryHeading();
            System.out.println("=== GYRO DIAGNOSTICS ===");
            System.out.println("Heading: " + heading.getDegrees() + "°");
            System.out.println("If heading changes without commands, gyro has drift");
            System.out.println("========================");
            SmartDashboard.putNumber("Gyro/Heading", heading.getDegrees());
        } catch (Exception e) {
            logError("Gyro diagnostic error: " + e.getMessage());
        }
    }

    @Override 
    public void periodic() {
        try {
            Pose2d pose = getPose();
            field2d.setRobotPose(pose);
            
            SmartDashboard.putNumber("Drive/Pose X (m)", pose.getX());
            SmartDashboard.putNumber("Drive/Pose Y (m)", pose.getY());
            SmartDashboard.putNumber("Drive/Pose Rotation (deg)", pose.getRotation().getDegrees());
            
            if (isInitialized()) {
                try {
                    SmartDashboard.putNumber("Drive/Heading (deg)", swerveDrive.getOdometryHeading().getDegrees());
                } catch (Exception e) {
                    SmartDashboard.putNumber("Drive/Heading (deg)", pose.getRotation().getDegrees());
                }
            } else {
                SmartDashboard.putNumber("Drive/Heading (deg)", pose.getRotation().getDegrees());
            }
            
            SmartDashboard.putNumber("Drive/X Velocity (m/s)", currentXSpeed);
            SmartDashboard.putNumber("Drive/Y Velocity (m/s)", currentYSpeed);
            SmartDashboard.putNumber("Drive/Rotation Velocity (rad/s)", currentRotSpeed);
            SmartDashboard.putNumber("Drive/Total Speed (m/s)", 
                Math.sqrt(currentXSpeed * currentXSpeed + currentYSpeed * currentYSpeed));
            
            updateModuleData();
            
            SmartDashboard.putBoolean("Drive/Initialized", isInitialized());
            SmartDashboard.putBoolean("Drive/Modules Zeroed", hasZeroed);
            SmartDashboard.putBoolean("Drive/Ready", isReadyToDrive());
            SmartDashboard.putBoolean("Drive/Simulation", RobotBase.isSimulation());
            
        } catch (Exception e) {
            logError("Periodic error: " + e.getMessage());
        }
    }
    
    /**
     * Update module data on SmartDashboard with error handling.
     */
    private void updateModuleData() {
        if (!isInitialized()) return;
        
        String[] moduleNames = {"FL", "FR", "BL", "BR"};
        
        try {
            var modules = swerveDrive.getModules();
            if (modules == null) return;
            
            for (int i = 0; i < modules.length && i < moduleNames.length; i++) {
                try {
                    if (modules[i] != null) {
                        double absPos = modules[i].getAbsolutePosition();
                        boolean isValid = !Double.isNaN(absPos) && !Double.isInfinite(absPos);
                        SmartDashboard.putNumber("Swerve/" + moduleNames[i] + " Angle", isValid ? absPos : 0.0);
                        SmartDashboard.putBoolean("Swerve/" + moduleNames[i] + " Encoder OK", isValid);
                    }
                } catch (Exception e) {
                    SmartDashboard.putBoolean("Swerve/" + moduleNames[i] + " Encoder OK", false);
                }
            }
        } catch (Exception e) {
            // Silently handle
        }
    }
}