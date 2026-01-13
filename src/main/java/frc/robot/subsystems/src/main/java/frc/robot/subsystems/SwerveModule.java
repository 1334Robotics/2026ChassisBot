package frc.robot.subsystems.src.main.java.frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class SwerveModule {

    // Max speed of robot (meters per second)
    private static final double kMaxSpeed = 4.5;

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final SparkAbsoluteEncoder turningEncoder;
    private final PIDController turningPIDController;

    public SwerveModule(int driveMotorId, int turningMotorId) {

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        turningEncoder = turningMotor.getAbsoluteEncoder();

        turningPIDController = new PIDController(0.5, 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Converts absolute encoder rotations → radians */
    private double getTurningRadians() {
        return turningEncoder.getPosition() * 2.0 * Math.PI;
    }

    /** Current module velocity & angle */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(getTurningRadians())
        );
    }

    /** Current module distance & angle (odometry) */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            new Rotation2d(getTurningRadians())
        );
    }

    /** Set desired state (speed + angle) */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Copy + optimize (NEW WPILib API)
        SwerveModuleState state = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle
        );

        state.optimize(new Rotation2d(getTurningRadians()));

        // Drive motor (open loop)
        driveMotor.set(state.speedMetersPerSecond / kMaxSpeed);

        // Turning motor PID
        double turnOutput = turningPIDController.calculate(
            getTurningRadians(),
            state.angle.getRadians()
        );

        turningMotor.set(turnOutput);
    }
}
