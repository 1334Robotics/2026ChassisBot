package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


  //Limelight vision subsystem for AprilTag detection.
 
public class LimelightVision extends SubsystemBase {
    // NetworkTable entries
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry botpose;
    private final NetworkTableEntry tl; // Pipeline latency
    private final NetworkTableEntry cl; // Capture latency
    private final NetworkTableEntry tid; // AprilTag ID
    
    // Cached values
    private double horizontalOffset = 0.0;
    private double verticalOffset = 0.0;
    private double targetArea = 0.0;
    private boolean hasTarget = false;
    private int currentTagId = -1;
    private double totalLatencyMs = 0.0;
    
    // Connection status
    private boolean isConnected = false;
    private double lastUpdateTime = 0.0;
    private static final double CONNECTION_TIMEOUT_SECONDS = 0.5;
    
    // Reference to drive subsystem
    private DriveSubsystem driveSubsystem = null;
    
    // Configuration
    private boolean autoUpdatePose = true;
    private double minTargetArea = 0.1; // Minimum area to consider valid
    private double maxPoseAmbiguity = 0.7; // Maximum ambiguity to accept pose
    
    // Field boundaries for pose validation (meters)
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.02;
    private static final double POSE_MARGIN = 0.5; // Allow small margin outside field
    
    public LimelightVision() {
        this("limelight");
        System.out.println("[LimelightVision] Initialized");
    }
    
    /**
     * Create a LimelightVision with a custom NetworkTable name.
     * @param tableName The name of the Limelight's NetworkTable (e.g., "limelight", "limelight-front")
     */
    public LimelightVision(String tableName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        
        // Get entries with null safety
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tv = limelightTable.getEntry("tv");
        botpose = limelightTable.getEntry("botpose_wpiblue"); // Use WPILib Blue alliance origin
        tl = limelightTable.getEntry("tl");
        cl = limelightTable.getEntry("cl");
        tid = limelightTable.getEntry("tid");
        
        // Initialize with safe defaults
        try {
            setLEDMode(LEDMode.OFF);
            setCameraMode(CameraMode.VISION);
            setPipeline(0);
        } catch (Exception e) {
            DriverStation.reportWarning("Limelight initialization warning: " + e.getMessage(), false);
        }
        
        SmartDashboard.putBoolean("Limelight/Auto Update Pose", autoUpdatePose);
        SmartDashboard.putNumber("Limelight/Min Target Area", minTargetArea);
    }
    
    /**
     * Set the drive subsystem to update with vision measurements.
     */
    public void setDriveSubsystem(DriveSubsystem drive) {
        this.driveSubsystem = drive;
    }
    
    /**
     * Enable or disable automatic pose updates to the drive subsystem.
     */
    public void setAutoUpdatePose(boolean enabled) {
        this.autoUpdatePose = enabled;
        SmartDashboard.putBoolean("Limelight/Auto Update Pose", autoUpdatePose);
    }
    
    /**
     * Set the minimum target area required to consider a detection valid.
     */
    public void setMinTargetArea(double area) {
        this.minTargetArea = Math.max(0.0, area);
        SmartDashboard.putNumber("Limelight/Min Target Area", minTargetArea);
    }
    
    @Override
    public void periodic() {
        try {
            updateValues();
            updateConnectionStatus();
            updateDashboard();
            
            // Automatically update drive subsystem with vision measurements
            if (autoUpdatePose && driveSubsystem != null && hasValidTarget()) {
                updateDriveWithVision();
            }
        } catch (Exception e) {
            DriverStation.reportError("LimelightVision periodic error: " + e.getMessage(), false);
        }
    }
    
    /**
     * Update cached values from NetworkTables.
     */
    private void updateValues() {
        horizontalOffset = tx.getDouble(0.0);
        verticalOffset = ty.getDouble(0.0);
        targetArea = ta.getDouble(0.0);
        hasTarget = tv.getDouble(0.0) == 1.0;
        currentTagId = (int) tid.getDouble(-1.0);
        
        // Calculate total latency
        double pipelineLatency = tl.getDouble(0.0);
        double captureLatency = cl.getDouble(0.0);
        totalLatencyMs = pipelineLatency + captureLatency;
    }
    
    /**
     * Check if we're receiving data from the Limelight.
     */
    private void updateConnectionStatus() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check if we're getting updates (tx changes or we have a target)
        double currentTx = tx.getDouble(Double.NaN);
        if (!Double.isNaN(currentTx)) {
            lastUpdateTime = currentTime;
            isConnected = true;
        } else if (currentTime - lastUpdateTime > CONNECTION_TIMEOUT_SECONDS) {
            isConnected = false;
        }
    }
    
    /**
     * Update SmartDashboard with current values.
     */
    private void updateDashboard() {
        SmartDashboard.putBoolean("Limelight/Connected", isConnected);
        SmartDashboard.putBoolean("Limelight/Has Target", hasTarget);
        SmartDashboard.putNumber("Limelight/TX", horizontalOffset);
        SmartDashboard.putNumber("Limelight/TY", verticalOffset);
        SmartDashboard.putNumber("Limelight/TA", targetArea);
        SmartDashboard.putNumber("Limelight/Tag ID", currentTagId);
        SmartDashboard.putNumber("Limelight/Latency (ms)", totalLatencyMs);
        
        if (hasTarget) {
            Pose2d pose = getRobotPose();
            SmartDashboard.putString("Limelight/Pose", 
                String.format("(%.2f, %.2f) @ %.1f°", 
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
        }
    }
    
    /**
     * Send vision measurement to drive subsystem with latency compensation.
     */
    private void updateDriveWithVision() {
        Pose2d visionPose = getRobotPose();
        
        // Validate pose before sending
        if (!isPoseValid(visionPose)) {
            SmartDashboard.putBoolean("Limelight/Pose Valid", false);
            return;
        }
        
        // Check target area is sufficient
        if (targetArea < minTargetArea) {
            SmartDashboard.putBoolean("Limelight/Pose Valid", false);
            return;
        }
        
        SmartDashboard.putBoolean("Limelight/Pose Valid", true);
        
        // Calculate timestamp with latency compensation
        double timestamp = Timer.getFPGATimestamp() - (totalLatencyMs / 1000.0);
        
        updateVisionPose(visionPose, timestamp);
    }
    
    /**
     * Update vision pose in the drive subsystem.
     */
    public void updateVisionPose(Pose2d visionPose, double timestamp) {
        if (driveSubsystem != null && visionPose != null) {
            driveSubsystem.addVisionMeasurement(visionPose, timestamp);
        }
    }
    
    /**
     * Check if a pose is within valid field boundaries.
     */
    private boolean isPoseValid(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        
        double x = pose.getX();
        double y = pose.getY();
        
        // Check if pose is within field boundaries (with margin)
        boolean xValid = x >= -POSE_MARGIN && x <= FIELD_LENGTH + POSE_MARGIN;
        boolean yValid = y >= -POSE_MARGIN && y <= FIELD_WIDTH + POSE_MARGIN;
        
        // Check for obviously invalid values
        boolean notZero = !(x == 0.0 && y == 0.0);
        boolean notNaN = !Double.isNaN(x) && !Double.isNaN(y);
        boolean notInfinite = !Double.isInfinite(x) && !Double.isInfinite(y);
        
        // Check pose ambiguity
        boolean ambiguityValid = maxPoseAmbiguity >= 0.0 && maxPoseAmbiguity <= 1.0;
        
        return xValid && yValid && notZero && notNaN && notInfinite && ambiguityValid;
    }
    
    /**
     * Check if we have a valid target that meets all criteria.
     */
    public boolean hasValidTarget() {
        return hasTarget && isConnected && targetArea >= minTargetArea;
    }
    
    /**
     * Check if the Limelight is connected.
     */
    public boolean isConnected() {
        return isConnected;
    }
    
    public double getHorizontalOffset() {
        return horizontalOffset;
    }
    
    public double getVerticalOffset() {
        return verticalOffset;
    }
    
    public double getTargetArea() {
        return targetArea;
    }
    
    public int getCurrentTagId() {
        return currentTagId;
    }
    
    public double getLatencyMs() {
        return totalLatencyMs;
    }
    
    /**
     * Calculate distance to target using trigonometry.
     */
    public double getDistanceToTarget(double targetHeightMeters, double cameraHeightMeters, double cameraMountAngleDegrees) {
        if (!hasValidTarget()) {
            return -1.0; // Return -1 to indicate no valid measurement
        }
        
        double angleToGoalDegrees = cameraMountAngleDegrees + verticalOffset;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        
        if (Math.abs(Math.tan(angleToGoalRadians)) < 0.001) {
            return -1.0; // Avoid division by near-zero
        }
        
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToGoalRadians);
    }
    
    /**
     * Get the robot pose from vision data.
     */
    public Pose2d getRobotPose() {
        try {
            double[] poseArray = botpose.getDoubleArray(new double[0]);
            
            if (poseArray == null || poseArray.length < 6) {
                return new Pose2d();
            }
            
            return new Pose2d(
                poseArray[0],
                poseArray[1],
                Rotation2d.fromDegrees(poseArray[5])
            );
        } catch (Exception e) {
            DriverStation.reportWarning("Error getting robot pose: " + e.getMessage(), false);
            return new Pose2d();
        }
    }
    
    /**
     * Get the full pose array including Z, roll, and pitch.
     */
    public double[] getFullPoseArray() {
        try {
            return botpose.getDoubleArray(new double[7]);
        } catch (Exception e) {
            return new double[7];
        }
    }
    
    public void setPipeline(int pipeline) {
        try {
            limelightTable.getEntry("pipeline").setNumber(Math.max(0, Math.min(9, pipeline)));
            SmartDashboard.putNumber("Limelight/Pipeline", pipeline);
        } catch (Exception e) {
            DriverStation.reportWarning("Error setting pipeline: " + e.getMessage(), false);
        }
    }
    
    public void setLEDMode(LEDMode mode) {
        try {
            limelightTable.getEntry("ledMode").setNumber(mode.getValue());
            SmartDashboard.putString("Limelight/LED Mode", mode.name());
        } catch (Exception e) {
            DriverStation.reportWarning("Error setting LED mode: " + e.getMessage(), false);
        }
    }
    
    public void setCameraMode(CameraMode mode) {
        try {
            limelightTable.getEntry("camMode").setNumber(mode.getValue());
            SmartDashboard.putString("Limelight/Camera Mode", mode.name());
        } catch (Exception e) {
            DriverStation.reportWarning("Error setting camera mode: " + e.getMessage(), false);
        }
    }
    
    /**
     * Take a snapshot for later analysis.
     */
    public void takeSnapshot() {
        try {
            limelightTable.getEntry("snapshot").setNumber(1);
        } catch (Exception e) {
            DriverStation.reportWarning("Error taking snapshot: " + e.getMessage(), false);
        }
    }
    
    /**
     * Reset snapshot mode.
     */
    public void resetSnapshot() {
        try {
            limelightTable.getEntry("snapshot").setNumber(0);
        } catch (Exception e) {
            DriverStation.reportWarning("Error resetting snapshot: " + e.getMessage(), false);
        }
    }
    
    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);
        
        private final int value;
        
        LEDMode(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    public enum CameraMode {
        VISION(0),
        DRIVER(1);
        
        private final int value;
        
        CameraMode(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
}
