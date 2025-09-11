package frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ModuleConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final int moduleId;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private final PIDController turnPIDController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};


    public SwerveModule(ModuleIO io, int id) {
        this.io = io;
        this.moduleId = id;

        driveDisconnectedAlert = new Alert(
                "Disconnected drive motor on module " + Integer.toString(moduleId) + ".",
                AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Disconnected turn motor on module " + Integer.toString(moduleId) + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert(
                "Disconnected turn encoder on module " + Integer.toString(moduleId) + ".",
                AlertType.kError);

    }

    /**
     * 
     * @return Module position in meters
     */
    public double getDrivePosition() {
        return (inputs.drivePositionRad / (2 * Math.PI)) * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public Rotation2d getTurningPosition() {
        return inputs.turnPosition;
    }

    /**
     * 
     * @return Module velocity in m/s
     */
    public double getDriveVelocity() {
        return (inputs.driveVelocityRadPerSec / (2 * Math.PI)) * ModuleConstants.kWheelDiameterMeters;
    }

    /**
     * 
     * @return Module steering velocity in m/s
     */
    public double getTurningVelocity() {
        return inputs.turnVelocityRadPerSec;
    }

    /**
     * 
     * @return Rotation2d
     */
    public Rotation2d getAbsoluteEncoderRad() {
        return inputs.turnAbsolutePosition;
    }

    // public void resetEncoders() {
    //     driveMotor.setPosition(0);
    //     // turningMotor.setPosition((getAbsoluteEncoderRad()) / (2 * Math.PI));
    // }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurningPosition());
    }

    public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward feedforward) {
        state.optimize(getTurningPosition());
        // state.cosineScale(inputs.turnPosition);

        io.setDriveVelocity(state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2));
        SmartDashboard.putNumber("state.angle", state.angle.getRotations());

        io.setTurnOpenLoop(turnPIDController.calculate(getState().angle.getRotations(), state.angle.getRotations()));
        // io.setTurnPosition();
    }

    public void stop() {
        io.setDriveOpenLoop(0);
        io.setTurnOpenLoop(0);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(moduleId), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * ModuleConstants.kDriveEncoderRot2Meter;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

}

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import frc.robot.Constants.ModuleConstants;

// public class SwerveModule {
// private final TalonFX driveMotor;
// private final TalonFX turningMotor;

// private final TalonFXConfiguration driveMotorConfig;
// private final TalonFXConfiguration turningMotorConfig;

// private final PIDController turningPidController;

// private final CANcoder absoluteEncoder;
// private final CANcoderConfiguration config;

// private final boolean absoluteEncoderReversed;
// private final double absoluteEncoderOffsetRad;

// public SwerveModule(int driveMotorId, int turningMotorId, InvertedValue
// driveMotorReversed, InvertedValue turningMotorReversed,
// int absoluteEncoderId, double absoluteEncoderOffset, boolean
// absoluteEncoderReversed) {

// this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
// this.absoluteEncoderReversed = absoluteEncoderReversed;
// absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
// config = new CANcoderConfiguration();

// driveMotor = new TalonFX(driveMotorId);
// driveMotorConfig = new TalonFXConfiguration();

// turningMotor = new TalonFX(turningMotorId);
// turningMotorConfig = new TalonFXConfiguration();

// driveMotorConfig.MotorOutput.withInverted(driveMotorReversed);
// driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
// driveMotorConfig.Feedback.FeedbackSensorSource =
// FeedbackSensorSourceValue.RotorSensor;

// turningMotorConfig.MotorOutput.withInverted(turningMotorReversed);
// turningMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
// turningMotorConfig.Feedback.FeedbackSensorSource =
// FeedbackSensorSourceValue.RotorSensor;

// config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
// config.MagnetSensor.SensorDirection =
// SensorDirectionValue.CounterClockwise_Positive;
// absoluteEncoder.getConfigurator().apply(config);

// turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
// turningPidController.enableContinuousInput(-Math.PI, Math.PI);

// turningMotor.getConfigurator().apply(turningMotorConfig);
// driveMotor.getConfigurator().apply(driveMotorConfig);

// resetEncoders();
// }

// public double getDrivePosition() {
// return driveMotor.getPosition().getValueAsDouble() *
// ModuleConstants.kTurningEncoderRot2Rad;
// }

// public double getTurningPosition() {
// return turningMotor.getPosition().getValueAsDouble() *
// ModuleConstants.kTurningEncoderRot2Rad;
// }

// public double getDriveVelocity() {
// return driveMotor.getVelocity().getValueAsDouble() *
// ModuleConstants.kDriveEncoderRPM2MeterPerSec;
// }

// public double getTurningVelocity() {
// return turningMotor.getVelocity().getValueAsDouble() *
// ModuleConstants.kTurningEncoderRPM2RadPerSec;
// }

// /*
// * Returns a double from -pi to pi.
// */
// public double getAbsoluteEncoderRad() {
// /*
// * double angle = absoluteEncoder.getVoltage() /
// RobotController.getVoltage5V();
// */
// // double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() *
// 2 * Math.PI/* + Math.PI / 2 */ ;
// double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 *
// Math.PI;
// /* angle *= 2.0 * Math.PI; */
// angle -= absoluteEncoderOffsetRad;
// return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
// }

// public void resetEncoders() {
// driveMotor.setPosition(0);
// turningMotor.setPosition((getAbsoluteEncoderRad()) / (2 * Math.PI));
// }

// public SwerveModulePosition getPosition() {
// return new SwerveModulePosition(getDrivePosition(), new
// Rotation2d(getTurningPosition()));
// }

// public SwerveModuleState getState() {
// return new SwerveModuleState(getDriveVelocity(), new
// Rotation2d(getTurningPosition()));
// }

// public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward
// feedforward) {
// if (Math.abs(feedforward.calculate(state.speedMetersPerSecond)) < 0.1) {
// stop();
// return;
// }
// state.optimize(getState().angle);
// /*
// * driveMotor.set(state.speedMetersPerSecond /
// DriveConstants.kPhysicalMaxSpeedMetersPerSecond); Original line
// */
// driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
// turningMotor.set(turningPidController.calculate(getTurningPosition(),
// state.angle.getRadians()));
// // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "]
// state", state.toString());
// }

// public void stop() {
// driveMotor.set(0);
// turningMotor.set(0);
// }
// }
