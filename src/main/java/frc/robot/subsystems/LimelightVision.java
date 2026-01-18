package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry botpose;
    
    private double horizontalOffset = 0.0;
    private double verticalOffset = 0.0;
    private double targetArea = 0.0;
    private boolean hasTarget = false;
    private DriveSubsystem driveSubsystem = null;
    
    public LimelightVision() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tv = limelightTable.getEntry("tv");
        botpose = limelightTable.getEntry("botpose");
        
        setLEDMode(LEDMode.OFF);
    }
    
    /**
     * Set the drive subsystem to update with vision measurements.
     * Call this from RobotContainer after both subsystems are created.
     */
    public void setDriveSubsystem(DriveSubsystem drive) {
        this.driveSubsystem = drive;
    }
    
    @Override
    public void periodic() {
        horizontalOffset = tx.getDouble(0.0);
        verticalOffset = ty.getDouble(0.0);
        targetArea = ta.getDouble(0.0);
        hasTarget = tv.getDouble(0.0) == 1.0;
        
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget);
        SmartDashboard.putNumber("Limelight TX", horizontalOffset);
        SmartDashboard.putNumber("Limelight TY", verticalOffset);
        SmartDashboard.putNumber("Limelight TA", targetArea);
        
        // Automatically update drive subsystem with vision measurements
        if (driveSubsystem != null && hasTarget) {
            Pose2d visionPose = getRobotPose();
            if (visionPose.getX() != 0.0 || visionPose.getY() != 0.0) {
                driveSubsystem.addVisionMeasurement(
                    visionPose, 
                    edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
                );
            }
        }
    }
    
    public boolean hasValidTarget() {
        return hasTarget;
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
    
    public double getDistanceToTarget(double targetHeightMeters, double cameraHeightMeters, double cameraMountAngleDegrees) {
        if (!hasValidTarget()) {
            return 0.0;
        }
        
        double angleToGoalDegrees = cameraMountAngleDegrees + verticalOffset;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToGoalRadians);
    }
    
    public Pose2d getRobotPose() {
        double[] poseArray = botpose.getDoubleArray(new double[6]);
        if (poseArray.length < 6) {
            return new Pose2d();
        }
        
        return new Pose2d(
            poseArray[0],
            poseArray[1],
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    public void setLEDMode(LEDMode mode) {
        limelightTable.getEntry("ledMode").setNumber(mode.getValue());
    }
    
    public void setCameraMode(CameraMode mode) {
        limelightTable.getEntry("camMode").setNumber(mode.getValue());
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
