package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
@SuppressWarnings("unused")


public class DriveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private final Field2d field2d;
    private boolean initialized = false;
    private boolean hasZeroed = false;
    private long lastErrorTime = 0;
    private static final long ERROR_THROTTLE_MS = 500;
    
    private double currentXSpeed = 0.0;
    private double currentYSpeed = 0.0;
    private double currentRotSpeed = 0.0;
    
    private double startupTime = 0;
    private static final double STARTUP_DELAY_SECONDS = 0.5;
    private static final double DEADZONE_THRESHOLD = 0.05;
    private static final double ROTATION_DEADZONE = 0.05;
    private static final double MAX_SPEED_LIMIT = 5.0;

    // Adjustable speed multiplier (changed via bumpers)
    private double speedMultiplier = DriveConstants.DEFAULT_SPEED_MULTIPLIER;

    public DriveSubsystem(File directory) {
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);
        
        currentXSpeed = 0.0;
        currentYSpeed = 0.0;
        currentRotSpeed = 0.0;
        
        initializeSwerve();
        
        startupTime = Timer.getFPGATimestamp();
        
        stop();
    }
    
    private void initializeSwerve() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "SWERVE");
            
            if (!swerveJsonDirectory.exists()) {
                logError("SWERVE directory not found: " + swerveJsonDirectory.getAbsolutePath());
                return;
            }
            
            File[] files = swerveJsonDirectory.listFiles();
            if (files != null) {
                logDebug("Files in SWERVE directory:");
                for (File f : files) {
                    logDebug("  - " + f.getName());
                }
            }

            logDebug("Creating SwerveDrive with max speed: " + DriveConstants.MAX_SPEED_MPS + " m/s");
            logDebug("Running in " + (RobotBase.isSimulation() ? "SIMULATION" : "REAL") + " mode");
            
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.MAX_SPEED_MPS);
            
            // Match YAGSL official example configuration:
            // 1. Heading correction should only be used while controlling via angle setpoint
            swerveDrive.setHeadingCorrection(false);
            // 2. Disable cosine compensation — causes discrepancies in simulation
            swerveDrive.setCosineCompensator(false);
            // 3. Correct for skew that gets worse as angular velocity increases
            swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
            // 4. Disable auto-synchronize to prevent jitter
            swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
            
            verifyModules();
            initialized = true;
            
            try {
                if (swerveDrive != null) {
                    var initialHeading = swerveDrive.getOdometryHeading();
                    logDebug("Initial gyro heading: " + initialHeading.getDegrees() + "°");
                    SmartDashboard.putNumber("Drive/Initial Heading", initialHeading.getDegrees());
                }
            } catch (Exception e) {
                logDebug("Could not read initial heading: " + e.getMessage());
            }
            
            zeroGyro();
            
            SmartDashboard.putBoolean("Drive/Initialized", true);
            logDebug("Swerve drive initialized successfully");
            
            // Configure PathPlanner after successful swerve init
            configurePathPlanner();
            
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
     * Configure PathPlanner's AutoBuilder for autonomous path following.
     * Must be called after YAGSL swerve drive is initialized.
     */
    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,                    // Pose supplier
                this::resetOdometry,              // Pose reset consumer
                this::getRobotRelativeSpeeds,     // ChassisSpeeds supplier (robot-relative)
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Drive consumer (robot-relative!)
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),   // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)    // Rotation PID
                ),
                config,
                () -> {
                    // Flip path for red alliance
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this                              // Subsystem requirement
            );
            logDebug("PathPlanner AutoBuilder configured successfully");
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Get the robot-relative chassis speeds (used by PathPlanner).
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        if (isInitialized()) {
            return swerveDrive.getRobotVelocity();
        }
        return new ChassisSpeeds();
    }

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
    
    private void logError(String message) {
        long now = System.currentTimeMillis();
        if (now - lastErrorTime > ERROR_THROTTLE_MS) {
            System.out.println("[DriveSubsystem ERROR] " + message);
            SmartDashboard.putString("Drive/Error", message);
            DriverStation.reportError(message, false);
            lastErrorTime = now;
        }
    }
    
    private void logDebug(String message) {
        System.out.println("[DriveSubsystem] " + message);
    }
    
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
    
    public void synchronizeModuleEncoders() {
        if (!isInitialized()) return;
        
        try {
            swerveDrive.synchronizeModuleEncoders();
            hasZeroed = true;
            SmartDashboard.putBoolean("Drive/Modules Synced", true);
            logDebug("Module encoders synchronized");
        } catch (Exception e) {
            DriverStation.reportWarning("synchronizeModuleEncoders error: " + e.getMessage(), true);
        }
    }
    
    public boolean isInitialized() {
        return initialized && swerveDrive != null;
    }
    
    private boolean isReadyToDrive() {
        return (Timer.getFPGATimestamp() - startupTime) >= STARTUP_DELAY_SECONDS;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    private double applyDeadzone(double value) {
        double absValue = Math.abs(value);
        if (absValue < DEADZONE_THRESHOLD) {
            return 0.0;
        }
        double sign = Math.signum(value);
        return sign * ((absValue - DEADZONE_THRESHOLD) / (1.0 - DEADZONE_THRESHOLD));
    }
    
    private double applyRotationDeadzone(double value) {
        double absValue = Math.abs(value);
        if (absValue < ROTATION_DEADZONE) {
            return 0.0;
        }
        double sign = Math.signum(value);
        return sign * ((absValue - ROTATION_DEADZONE) / (1.0 - ROTATION_DEADZONE));
    }

    /**
     * Clamp a velocity component so the robot cannot drive outside the field.
     * When the robot is near or beyond a boundary, velocity pushing further out is zeroed.
     * The margin constant controls how close to the wall the brake engages.
     */
    private double clampFieldVelocity(double velocity, double position, double minBound, double maxBound) {
        final double MARGIN = FieldConstants.FIELD_MARGIN; // 0.5 m from wall
        if (position <= minBound + MARGIN && velocity < 0) return 0.0;
        if (position >= maxBound - MARGIN && velocity > 0) return 0.0;
        return velocity;
    }

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
    
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                                DoubleSupplier rotationX) {
        return run(() -> {
            if (!isInitialized()) {
                return;
            }
            
            try {
                double xInput = translationX.getAsDouble();
                double yInput = translationY.getAsDouble();
                double rotInput = rotationX.getAsDouble();
                
                // Apply deadzone
                xInput = applyDeadzone(xInput);
                yInput = applyDeadzone(yInput);
                rotInput = applyRotationDeadzone(rotInput);
                
                // Scale to velocity (m/s and rad/s), applying speed multiplier
                currentXSpeed = xInput * swerveDrive.getMaximumChassisVelocity() * speedMultiplier;
                currentYSpeed = yInput * swerveDrive.getMaximumChassisVelocity() * speedMultiplier;
                currentRotSpeed = Math.pow(rotInput, 3) * swerveDrive.getMaximumChassisAngularVelocity() * speedMultiplier;
                
                // Field boundary protection — prevent driving off the field
                // Velocities here are field-relative (X = downfield, Y = cross-field)
                Pose2d pose = swerveDrive.getPose();
                currentXSpeed = clampFieldVelocity(currentXSpeed, pose.getX(), 0, FieldConstants.FIELD_LENGTH);
                currentYSpeed = clampFieldVelocity(currentYSpeed, pose.getY(), 0, FieldConstants.FIELD_WIDTH);

                // Drive using YAGSL — field-relative, closed-loop
                swerveDrive.drive(
                    new Translation2d(currentXSpeed, currentYSpeed),
                    currentRotSpeed,
                    true,
                    false);
                
            } catch (Exception e) {
                logError("Drive command error: " + e.getMessage());
                stop();
            }
        }).withName("DriveCommand");
    }
    
    // Removed deprecated driveFieldOrientedSafe - all drive paths now use YAGSL consistently

    private long diagPrintCounter = 0;

    /**
     * Diagnostic command: drives the robot at a fixed 1 m/s forward using robot-relative ChassisSpeeds.
     * Logs detailed module state info to identify spinning root cause.
     */
    public Command testDriveForwardCommand() {
        return run(() -> {
            if (isInitialized()) {
                ChassisSpeeds desired = new ChassisSpeeds(1.0, 0.0, 0.0);
                
                // Drive robot-relative, closed-loop
                swerveDrive.drive(desired);
                
                // Log what kinematics WANTS vs what modules ACTUALLY report
                var desiredStates = swerveDrive.kinematics.toSwerveModuleStates(desired);
                var modules = swerveDrive.getModules();
                String[] names = {"FL", "FR", "BL", "BR"};
                boolean shouldPrint = (diagPrintCounter++ % 50 == 0); // Print every 1 second at 50Hz
                
                if (shouldPrint) {
                    System.out.println("=== DIAG: ChassisSpeeds(1,0,0) robot-relative ===");
                    System.out.printf("  Heading: %.2f deg%n", swerveDrive.getOdometryHeading().getDegrees());
                }
                
                for (int i = 0; i < modules.length; i++) {
                    var actual = modules[i].getState();
                    SmartDashboard.putNumber("Diag/" + names[i] + "/DesiredAngle", desiredStates[i].angle.getDegrees());
                    SmartDashboard.putNumber("Diag/" + names[i] + "/DesiredSpeed", desiredStates[i].speedMetersPerSecond);
                    SmartDashboard.putNumber("Diag/" + names[i] + "/ActualAngle", actual.angle.getDegrees());
                    SmartDashboard.putNumber("Diag/" + names[i] + "/ActualSpeed", actual.speedMetersPerSecond);
                    SmartDashboard.putNumber("Diag/" + names[i] + "/AngleError", 
                        desiredStates[i].angle.getDegrees() - actual.angle.getDegrees());
                    
                    // Also log maple-sim direct state if available (via YAGSL sim wrapper)
                    var simMod = modules[i].getSimModule();
                    if (simMod != null) {
                        var simState = simMod.getState();
                        SmartDashboard.putNumber("Diag/" + names[i] + "/SimAngle", simState.angle.getDegrees());
                        SmartDashboard.putNumber("Diag/" + names[i] + "/SimSpeed", simState.speedMetersPerSecond);
                    }
                    
                    if (shouldPrint) {
                        System.out.printf("  %s: desired(%.1f°, %.2f m/s) actual(%.1f°, %.2f m/s) err=%.1f°%n",
                            names[i],
                            desiredStates[i].angle.getDegrees(), desiredStates[i].speedMetersPerSecond,
                            actual.angle.getDegrees(), actual.speedMetersPerSecond,
                            desiredStates[i].angle.getDegrees() - actual.angle.getDegrees());
                    }
                }
                var velocity = swerveDrive.getRobotVelocity();
                SmartDashboard.putNumber("Diag/ActualVx", velocity.vxMetersPerSecond);
                SmartDashboard.putNumber("Diag/ActualVy", velocity.vyMetersPerSecond);
                SmartDashboard.putNumber("Diag/ActualOmega", velocity.omegaRadiansPerSecond);
                SmartDashboard.putNumber("Diag/Heading", swerveDrive.getOdometryHeading().getDegrees());
                
                if (shouldPrint) {
                    System.out.printf("  Velocity: vx=%.2f vy=%.2f omega=%.3f rad/s%n",
                        velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);
                    System.out.printf("  Pose: (%.2f, %.2f) @ %.1f°%n",
                        swerveDrive.getPose().getX(), swerveDrive.getPose().getY(),
                        swerveDrive.getPose().getRotation().getDegrees());
                }
            }
        }).withName("TestDriveForward");
    }

    public void stop() {
        currentXSpeed = 0.0;
        currentYSpeed = 0.0;
        currentRotSpeed = 0.0;
        
        // Always tell YAGSL to stop — it handles sim internally
        if (isInitialized()) {
            try {
                swerveDrive.drive(new Translation2d(0, 0), 0.0, false, false);
            } catch (Exception e) {
                logError("Stop error: " + e.getMessage());
            }
        }
        SmartDashboard.putString("Drive/Status", "Stopped");
    }

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
    
    public void driveFieldOriented(ChassisSpeeds velocity) {
        if (velocity == null) {
            velocity = new ChassisSpeeds(0, 0, 0);
        }
        
        currentXSpeed = velocity.vxMetersPerSecond;
        currentYSpeed = velocity.vyMetersPerSecond;
        currentRotSpeed = velocity.omegaRadiansPerSecond;
        
        // Always use YAGSL — it handles sim internally via maple-sim
        if (isInitialized()) {
            try {
                swerveDrive.driveFieldOriented(velocity);
            } catch (Exception e) {
                logError("driveFieldOriented error: " + e.getMessage());
            }
        }
    }

    /**
     * Drive using robot-relative chassis speeds.
     * Used by PathPlanner's AutoBuilder — PathPlanner outputs robot-relative speeds.
     */
    public void driveRobotRelative(ChassisSpeeds velocity) {
        if (velocity == null) {
            velocity = new ChassisSpeeds(0, 0, 0);
        }
        
        currentXSpeed = velocity.vxMetersPerSecond;
        currentYSpeed = velocity.vyMetersPerSecond;
        currentRotSpeed = velocity.omegaRadiansPerSecond;
        
        if (isInitialized()) {
            try {
                swerveDrive.drive(velocity);
            } catch (Exception e) {
                logError("driveRobotRelative error: " + e.getMessage());
            }
        }
    }

    public Pose2d getPose() {
        // Always use YAGSL's pose — it tracks sim state internally
        if (isInitialized()) {
            try {
                return swerveDrive.getPose();
            } catch (Exception e) {
                logError("getPose error: " + e.getMessage());
            }
        }
        return new Pose2d();
    }

    public void resetOdometry(Pose2d pose) {
        if (pose == null) {
            pose = new Pose2d();
        }
        
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

    public Field2d getField2d() {
        return field2d;
    }

    public double getMaxSpeed() {
        return DriveConstants.MAX_SPEED_MPS;
    }

    /** Increase the teleop speed multiplier by one step. */
    public void increaseSpeed() {
        speedMultiplier = Math.min(DriveConstants.MAX_SPEED_MULTIPLIER,
                                   speedMultiplier + DriveConstants.SPEED_MULTIPLIER_STEP);
        SmartDashboard.putNumber("Drive/Speed Multiplier", speedMultiplier);
        logDebug("Speed multiplier: " + String.format("%.0f%%", speedMultiplier * 100));
    }

    /** Decrease the teleop speed multiplier by one step. */
    public void decreaseSpeed() {
        speedMultiplier = Math.max(DriveConstants.MIN_SPEED_MULTIPLIER,
                                   speedMultiplier - DriveConstants.SPEED_MULTIPLIER_STEP);
        SmartDashboard.putNumber("Drive/Speed Multiplier", speedMultiplier);
        logDebug("Speed multiplier: " + String.format("%.0f%%", speedMultiplier * 100));
    }

    /** Get the current speed multiplier (0.0–1.0). */
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

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
    
    public void diagnoseMotorInversions() {
        if (!isInitialized()) {
            System.out.println("Drive not initialized");
            return;
        }
        
        System.out.println("=== MOTOR INVERSION DIAGNOSTICS ===");
        String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
        
        try {
            var modules = swerveDrive.getModules();
            for (int i = 0; i < modules.length && i < moduleNames.length; i++) {
                if (modules[i] != null) {
                    System.out.println("\n" + moduleNames[i] + ":");
                    System.out.println("  Absolute Position: " + modules[i].getAbsolutePosition() + "°");
                    SmartDashboard.putNumber("Diag/" + moduleNames[i] + " Abs Pos", 
                        modules[i].getAbsolutePosition());
                }
            }
            
            // Check gyro heading - THIS IS OFTEN THE CAUSE OF ROTATION ISSUES
            var heading = swerveDrive.getOdometryHeading();
            System.out.println("\n=== GYRO HEADING CHECK ===");
            System.out.println("Current Heading: " + heading.getDegrees() + "°");
            System.out.println("If heading is NOT 0° when robot faces forward, wheels will spin!");
            System.out.println("Press POV Up to zero gyro if needed.");
            
            System.out.println("\n=== EXPECTED BEHAVIOR ===");
            System.out.println("1. When driving forward (Y+), all modules should point forward");
            System.out.println("2. When driving left (X+), all modules should point left");
            System.out.println("3. When rotating CCW, modules should form an X pattern");
            System.out.println("4. If robot spins unexpectedly:");
            System.out.println("   - Check if gyro heading is 0 when facing forward");
            System.out.println("   - Check 'invertedIMU' in swervedrive.json");
            System.out.println("   - Try pressing POV Up to zero gyro");
            System.out.println("5. If robot drives wrong direction, check drive motor 'inverted' settings");
            System.out.println("===================================");
            
        } catch (Exception e) {
            logError("Motor inversion diagnostic error: " + e.getMessage());
        }
    }

    @Override 
    public void periodic() {
        if (isInitialized()) {
            try {
                Pose2d pose = swerveDrive.getPose();
                field2d.setRobotPose(pose);

                // Essential telemetry — always available on dashboard
                SmartDashboard.putNumber("Drive/X (m)", Math.round(pose.getX() * 100.0) / 100.0);
                SmartDashboard.putNumber("Drive/Y (m)", Math.round(pose.getY() * 100.0) / 100.0);
                SmartDashboard.putNumber("Drive/Heading (deg)",
                    Math.round(pose.getRotation().getDegrees() * 10.0) / 10.0);
                SmartDashboard.putNumber("Drive/Speed Multiplier", speedMultiplier);

                // Battery voltage — critical for knowing when to swap
                double voltage = RobotController.getBatteryVoltage();
                SmartDashboard.putNumber("Robot/Battery (V)", Math.round(voltage * 100.0) / 100.0);
                if (voltage < 11.5) {
                    SmartDashboard.putString("Robot/Battery Warning", "LOW BATTERY!");
                } else {
                    SmartDashboard.putString("Robot/Battery Warning", "OK");
                }
            } catch (Exception e) {
                // silently handle
            }
        }
    }
    
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