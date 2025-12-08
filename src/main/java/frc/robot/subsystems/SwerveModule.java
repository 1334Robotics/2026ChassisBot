import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.math.util.Units;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;


public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;
    private CANcoder    absoluteEncoder;
    private SparkMaxPIDController drivingPIDController;
    private SparkMaxPIDController turningPIDController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
    {
        driveMotor = new CANSparkMax(driveMotorCANID);
        steerMotor = new CANSparkMax(steerMotorCANID);
        absoluteEncoder = new CANcoder(cancoderCANID);
        
        // Get the PID Controllers
        drivingPIDController = driveMotor.getPIDController();
        turningPIDController = steerMotor.getPIDController();
        
        // Get the encoders
        driveEncoder = driveMotor.getEncoder():
        steerEncoder = steerMotor.getEncoder();
        
        // Reset everything to factory default
        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        
        // Continue configuration here..
        
        // CANcoder Configuration
        CANcoderConfigurator cfg = encoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        // Steering Motor Configuration
        steerMotor.setInverted(false);
        turningPIDController.setFeedbackDevice(steerEncoder);
        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        steerEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        steerEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput(90);
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        turningPIDController.setP(ModuleConstants.kTurningP);
        turningPIDController.setI(ModuleConstants.kTurningI);
        turningPIDController.setD(ModuleConstants.kTurningD);
        turningPIDController.setFF(ModuleConstants.kTurningFF);
        
        // Drive Motor Configuration
        driveMotor.setInverted(false);
        drivingPIDController.setFeedbackDevice(driveEncoder);
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);        
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        drivingPIDController.setP(ModuleConstants.kDrivingP);
        drivingPIDController.setI(ModuleConstants.kDrivingI);
        drivingPIDController.setD(ModuleConstants.kDrivingD);
        drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        
        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        driveMotor.burnFlash();
        steerMotor.burnFlash();
          
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(encoder.getAbsolutePosition().refresh().getValue() * 360);
    }
    
    
    /**
    Get the distance in meters.
    */
    public double getDistance()
    {
        return driveEncoder.getPosition();
    }
    
    /**
    Get the angle.
    */
    public Rotation2d getAngle()
    {
          return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state)
    {
          turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
          drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

}