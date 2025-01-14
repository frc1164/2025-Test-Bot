package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final SparkMaxConfig driveMotorConfig;
    private final SparkMaxConfig turningMotorConfig;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration config;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        config = new CANcoderConfiguration();

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        driveMotorConfig = new SparkMaxConfig();

        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        turningMotorConfig = new SparkMaxConfig();

        driveMotorConfig
                .inverted(driveMotorReversed)
                .idleMode(IdleMode.kBrake);
        driveMotorConfig.encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningMotorConfig
                .inverted(turningMotorReversed)
                .idleMode(IdleMode.kBrake);
        turningMotorConfig.encoder
                .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveMotor.configure(driveMotorConfig, null, null);
        turningMotor.configure(turningMotorConfig, null, null);

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        /*
         * double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
         */
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI + Math.PI / 2;
        /* angle *= 2.0 * Math.PI; */
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward feedforward) {
        if (Math.abs(feedforward.calculate(state.speedMetersPerSecond)) < 0.1) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        /*
         * driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); Original line
         */
        driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
