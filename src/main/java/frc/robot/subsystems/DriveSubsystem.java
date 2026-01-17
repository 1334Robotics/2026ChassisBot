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
    
    public DriveSubsystem(File directory) {
        // Initialize Field2d first (always works)
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);
        
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "SWERVE");
            
            // Check if directory exists
            if (!swerveJsonDirectory.exists()) {
                DriverStation.reportError("SWERVE directory not found: " + swerveJsonDirectory.getAbsolutePath(), false);
                SmartDashboard.putString("Drive/Error", "SWERVE directory not found");
                return;
            }
            
            // List all files for debugging
            File[] files = swerveJsonDirectory.listFiles();
            if (files != null) {
                System.out.println("=== SWERVE Directory Contents ===");
                for (File f : files) {
                    System.out.println("  " + f.getName() + " (isFile: " + f.isFile() + ", isDir: " + f.isDirectory() + ")");
                }
                System.out.println("=================================");
            }
            
            // Check modules subdirectory
            File modulesDir = new File(swerveJsonDirectory, "modules");
            if (modulesDir.exists() && modulesDir.isDirectory()) {
                File[] moduleFiles = modulesDir.listFiles();
                if (moduleFiles != null) {
                    System.out.println("=== SWERVE/modules Contents ===");
                    for (File f : moduleFiles) {
                        System.out.println("  " + f.getName());
                    }
                    System.out.println("=================================");
                }
            }
            
            System.out.println("Creating SwerveDrive with max speed: " + DriveConstants.MAX_SPEED_MPS + " m/s");
            System.out.println("Running in " + (RobotBase.isSimulation() ? "SIMULATION" : "REAL") + " mode");
            
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.MAX_SPEED_MPS);
            
            // Verify all modules loaded
            var modules = swerveDrive.getModules();
            System.out.println("========================================");
            System.out.println("SUCCESS: Loaded " + modules.length + " swerve modules");
            System.out.println("========================================");
            SmartDashboard.putNumber("Drive/Module Count", modules.length);
            
            // Print details of each module
            for (int i = 0; i < modules.length; i++) {
                if (modules[i] != null) {
                    System.out.println("Module " + i + " initialized successfully");
                    SmartDashboard.putBoolean("Drive/Module " + i + " OK", true);
                } else {
                    System.out.println("WARNING: Module " + i + " is NULL!");
                    SmartDashboard.putBoolean("Drive/Module " + i + " OK", false);
                }
            }
            
            if (modules.length != 4) {
                DriverStation.reportError("Expected 4 modules, got " + modules.length, false);
                System.out.println("ERROR: Only " + modules.length + " modules loaded!");
            } else {
                System.out.println("All 4 modules loaded correctly!");
            }
            
            initialized = true;
            
            // Zero the gyro on startup to prevent initial spinning
            zeroGyro();
            
            // Synchronize module encoders
            synchronizeModuleEncoders();
            
            SmartDashboard.putBoolean("Drive/Initialized", true);
            
        } catch (Exception e) {
            System.out.println("========================================");
            System.out.println("FATAL ERROR during swerve initialization:");
            System.out.println("Message: " + e.getMessage());
            System.out.println("Stack trace:");
            e.printStackTrace();
            System.out.println("========================================");
            DriverStation.reportError("Failed to initialize swerve drive: " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putString("Drive/Error", "Init failed: " + e.getMessage());
            SmartDashboard.putBoolean("Drive/Initialized", false);
            
            // In simulation, we can continue without hardware
            if (RobotBase.isSimulation()) {
                DriverStation.reportWarning("Running in simulation mode without full swerve hardware", false);
            }
        }
        
        // Put initial values on dashboard
        SmartDashboard.putNumber("Drive/Max Speed (m/s)", DriveConstants.MAX_SPEED_MPS);
        SmartDashboard.putNumber("Drive/Max Angular Velocity (rad/s)", DriveConstants.MAX_ANGULAR_VELOCITY);
        
        lastUpdateTime = Timer.getFPGATimestamp();
        startupTime = Timer.getFPGATimestamp();
    }
    
    /**
     * Zero the gyroscope heading.
     */
    public void zeroGyro() {
        if (isInitialized()) {
            try {
                swerveDrive.zeroGyro();
                SmartDashboard.putBoolean("Drive/Gyro Zeroed", true);
            } catch (Exception e) {
                DriverStation.reportError("zeroGyro error: " + e.getMessage(), false);
            }
        }
    }
    
    /**
     * Synchronize module encoders to absolute encoders.
     */
    public void synchronizeModuleEncoders() {
        if (isInitialized()) {
            try {
                swerveDrive.synchronizeModuleEncoders();
                hasZeroed = true;
                SmartDashboard.putBoolean("Drive/Modules Synced", true);
            } catch (Exception e) {
                DriverStation.reportWarning("synchronizeModuleEncoders error: " + e.getMessage(), false);
            }
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
     * Command to drive the robot using joystick inputs.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                                DoubleSupplier rotationX, DoubleSupplier headingY) {
        return run(() -> {
            try {
                // Don't drive during startup delay
                if (!isReadyToDrive()) {
                    stop();
                    return;
                }
                
                // Get and validate joystick inputs (already has deadband applied from RobotContainer)
                double xInput = clamp(translationX.getAsDouble(), -1.0, 1.0);
                double yInput = clamp(translationY.getAsDouble(), -1.0, 1.0);
                double rotInput = clamp(rotationX.getAsDouble(), -1.0, 1.0);
                
                // Convert to velocities
                currentXSpeed = xInput * DriveConstants.MAX_SPEED_MPS;
                currentYSpeed = yInput * DriveConstants.MAX_SPEED_MPS;
                currentRotSpeed = rotInput * DriveConstants.MAX_ANGULAR_VELOCITY;
                
                // Create and apply chassis speeds
                ChassisSpeeds speeds = new ChassisSpeeds(currentXSpeed, currentYSpeed, currentRotSpeed);
                driveFieldOrientedSafe(speeds);
                
            } catch (Exception e) {
                DriverStation.reportError("Drive command error: " + e.getMessage(), false);
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
                DriverStation.reportError("driveFieldOriented error: " + e.getMessage(), false);
            }
        } else if (RobotBase.isSimulation()) {
            // Simulate movement in simulation mode
            updateSimulatedPose(speeds);
        }
    }
    
    /**
     * Update simulated pose when hardware isn't available.
     */
    private void updateSimulatedPose(ChassisSpeeds speeds) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        
        // Clamp dt to reasonable values
        dt = clamp(dt, 0.0, 0.1);
        
        // Field-oriented speeds are already in the field frame, so use them directly
        double dx = speeds.vxMetersPerSecond * dt;
        double dy = -speeds.vyMetersPerSecond * dt;
        double dTheta = speeds.omegaRadiansPerSecond * dt;
        
        double newX = simPose.getX() + dx;
        double newY = simPose.getY() + dy;
        Rotation2d newRotation = simPose.getRotation().plus(new Rotation2d(dTheta));
        
        simPose = new Pose2d(newX, newY, newRotation);
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
                DriverStation.reportError("Stop error: " + e.getMessage(), false);
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
                DriverStation.reportError("Lock error: " + e.getMessage(), false);
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
            DriverStation.reportError("seedForwards error: " + e.getMessage(), false);
        }
    }
    
    /**
     * Drive the robot given a chassis field oriented velocity.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        if (velocity == null) {
            velocity = new ChassisSpeeds(0, 0, 0);
        }
        
        // Update tracked speeds
        currentXSpeed = velocity.vxMetersPerSecond;
        currentYSpeed = velocity.vyMetersPerSecond;
        currentRotSpeed = velocity.omegaRadiansPerSecond;
        
        driveFieldOrientedSafe(velocity);
    }

    /**
     * Get the current pose of the robot.
     */
    public Pose2d getPose() {
        if (isInitialized()) {
            try {
                return swerveDrive.getPose();
            } catch (Exception e) {
                DriverStation.reportError("getPose error: " + e.getMessage(), false);
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
        
        simPose = pose; // Always update sim pose
        
        if (isInitialized()) {
            try {
                swerveDrive.resetOdometry(pose);
            } catch (Exception e) {
                DriverStation.reportError("resetOdometry error: " + e.getMessage(), false);
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
            DriverStation.reportError("addVisionMeasurement error: " + e.getMessage(), false);
        }
    }

    /**
     * Get the Field2d object for adding trajectories.
     */
    public Field2d getField2d() {
        return field2d;
    }
    
    /**
     * Update the Field2d display with current robot pose and module directions.
     */
    public void updateField2dDisplay() {
        Pose2d pose = getPose();
        field2d.setRobotPose(pose);
    }

    /**
     * Get the maximum speed in meters per second.
     */
    public double getMaxSpeed() {
        return DriveConstants.MAX_SPEED_MPS;
    }

    /**
     * Print current absolute encoder positions for calibration.
     * Point all modules forward, then call this to get offset values.
     */
    public void printEncoderOffsets() {
        if (!isInitialized()) {
            System.out.println("Drive not initialized - cannot read encoders");
            return;
        }
        
        String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
        System.out.println("========== ENCODER OFFSETS ==========");
        System.out.println("Point all wheels forward, then use these as offsets:");
        
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
            System.out.println("Error reading encoders: " + e.getMessage());
        }
        System.out.println("======================================");
    }

    @Override 
    public void periodic() {
        try {
            // Update Field2d with current robot pose
            updateField2dDisplay();
            Pose2d pose = getPose();
            field2d.setRobotPose(pose);
            
            // ==================== POSE DATA ====================
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
            
            // ==================== VELOCITY DATA ====================
            SmartDashboard.putNumber("Drive/X Velocity (m/s)", currentXSpeed);
            SmartDashboard.putNumber("Drive/Y Velocity (m/s)", currentYSpeed);
            SmartDashboard.putNumber("Drive/Rotation Velocity (rad/s)", currentRotSpeed);
            SmartDashboard.putNumber("Drive/Total Speed (m/s)", 
                Math.sqrt(currentXSpeed * currentXSpeed + currentYSpeed * currentYSpeed));
            
            // ==================== MODULE DATA ====================
            updateModuleData();
            
            // ==================== STATUS ====================
            SmartDashboard.putBoolean("Drive/Initialized", isInitialized());
            SmartDashboard.putBoolean("Drive/Modules Zeroed", hasZeroed);
            SmartDashboard.putBoolean("Drive/Ready", isReadyToDrive());
            SmartDashboard.putBoolean("Drive/Simulation", RobotBase.isSimulation());
            
        } catch (Exception e) {
            DriverStation.reportError("DriveSubsystem periodic error: " + e.getMessage(), false);
        }
    }
    
    /**
     * Update module data on SmartDashboard with error handling.
     */
    private void updateModuleData() {
        if (!isInitialized()) {
            return;
        }
        
        String[] moduleNames = {"FL", "FR", "BL", "BR"};
        
        try {
            var modules = swerveDrive.getModules();
            if (modules == null) {
                return;
            }
            
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
            // Silently handle - modules may not be available in simulation
        }
    }
}