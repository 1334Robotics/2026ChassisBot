package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;

/**
 * Individual swerve module controller (not used with YAGSL, kept for reference).
 * Each module controls one drive motor and one turning motor for swerve drive.
 * 
 * Motor Layout:
 * - Drive Motor: Controls forward/backward speed (brushless)
 * - Turning Motor: Controls wheel direction (brushless)
 * - Turning Encoder: Absolute encoder for wheel angle feedback
 */
public class SwerveModule {
    private static final double kMaxSpeed = 4.5; // m/s
    private static final double ENCODER_CONVERSION = 2.0 * Math.PI; // radians per rotation

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private final SparkAbsoluteEncoder turningEncoder;
    private final PIDController turningPIDController;

    public SwerveModule(int driveMotorId, int turningMotorId) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        turningEncoder = turningMotor.getAbsoluteEncoder();
        
        // PID for wheel angle control
        turningPIDController = new PIDController(0.5, 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Convert encoder position to radians.
     */
    private double getTurningRadians() {
        return turningEncoder.getPosition() * ENCODER_CONVERSION;
    }

    /**
     * Get current state of this module (velocity and angle).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(getTurningRadians())
        );
    }

    /**
     * Get current position of this module (distance traveled and angle).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            new Rotation2d(getTurningRadians())
        );
    }

    /**
     * Set desired state for this module with automatic optimization.
     * Optimizes to minimize turning (never rotates more than 90 degrees).
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if (desiredState == null) {
            stop();
            return;
        }
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState,
            new Rotation2d(getTurningRadians())
        );

        // Set drive motor power (normalized 0-1)
        driveMotor.set(optimizedState.speedMetersPerSecond / kMaxSpeed);

        // Calculate and set turning motor power
        double turnOutput = turningPIDController.calculate(
            getTurningRadians(),
            optimizedState.angle.getRadians()
        );

        turningMotor.set(turnOutput);
    }
    
    /**
     * Stop all motion on this module.
     */
    private void stop() {
        driveMotor.set(0.0);
        turningMotor.set(0.0);
    }
}
