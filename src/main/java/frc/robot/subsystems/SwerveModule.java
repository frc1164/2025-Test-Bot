package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final TalonFXConfiguration driveMotorConfig;
    private final TalonFXConfiguration turningMotorConfig;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration config;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, InvertedValue driveMotorReversed, InvertedValue turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        config = new CANcoderConfiguration();


        driveMotor = new TalonFX(driveMotorId);
        driveMotorConfig = new TalonFXConfiguration();


        turningMotor = new TalonFX(turningMotorId);
        turningMotorConfig = new TalonFXConfiguration();

        driveMotorConfig.MotorOutput.withInverted(driveMotorReversed);
        driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        driveMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


        turningMotorConfig.MotorOutput.withInverted(turningMotorReversed);
        turningMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        // turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turningMotorConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoderId;
        turningMotorConfig.Feedback.RotorToSensorRatio = ModuleConstants.kTurningMotorGearRatio;


        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        turningMotor.getConfigurator().apply(turningMotorConfig);
        driveMotor.getConfigurator().apply(driveMotorConfig);


        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble() * 2 * Math.PI - absoluteEncoderOffsetRad;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    /*
     * Returns a double from -pi to pi.
     */
    public double getAbsoluteEncoderRad() {
        /*
         * double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
         */

        // double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI/* + Math.PI / 2 */ ;
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        /* angle *= 2.0 * Math.PI; */
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        // turningMotor.setPosition((getAbsoluteEncoderRad()) / (2 * Math.PI));
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
//     private final TalonFX driveMotor;
//     private final TalonFX turningMotor;

//     private final TalonFXConfiguration driveMotorConfig;
//     private final TalonFXConfiguration turningMotorConfig;

//     private final PIDController turningPidController;

//     private final CANcoder absoluteEncoder;
//     private final CANcoderConfiguration config;

//     private final boolean absoluteEncoderReversed;
//     private final double absoluteEncoderOffsetRad;

//     public SwerveModule(int driveMotorId, int turningMotorId, InvertedValue driveMotorReversed, InvertedValue turningMotorReversed,
//             int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

//         this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
//         this.absoluteEncoderReversed = absoluteEncoderReversed;
//         absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
//         config = new CANcoderConfiguration();

//         driveMotor = new TalonFX(driveMotorId);
//         driveMotorConfig = new TalonFXConfiguration();

//         turningMotor = new TalonFX(turningMotorId);
//         turningMotorConfig = new TalonFXConfiguration();

//         driveMotorConfig.MotorOutput.withInverted(driveMotorReversed);
//         driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
//         driveMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


//         turningMotorConfig.MotorOutput.withInverted(turningMotorReversed);
//         turningMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
//         turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


//         config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
//         config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
//         absoluteEncoder.getConfigurator().apply(config);

//         turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
//         turningPidController.enableContinuousInput(-Math.PI, Math.PI);

//         turningMotor.getConfigurator().apply(turningMotorConfig);
//         driveMotor.getConfigurator().apply(driveMotorConfig);


//         resetEncoders();
//     }

//     public double getDrivePosition() {
//         return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRot2Rad;
//     }

//     public double getTurningPosition() {
//         return turningMotor.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRot2Rad;
//     }

//     public double getDriveVelocity() {
//         return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
//     }

//     public double getTurningVelocity() {
//         return turningMotor.getVelocity().getValueAsDouble() * ModuleConstants.kTurningEncoderRPM2RadPerSec;
//     }

//     /*
//      * Returns a double from -pi to pi.
//      */
//     public double getAbsoluteEncoderRad() {
//         /*
//          * double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
//          */
//         // double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI/* + Math.PI / 2 */ ;
//         double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
//         /* angle *= 2.0 * Math.PI; */
//         angle -= absoluteEncoderOffsetRad;
//         return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
//     }

//     public void resetEncoders() {
//         driveMotor.setPosition(0);
//         turningMotor.setPosition((getAbsoluteEncoderRad()) / (2 * Math.PI));
//     }

//     public SwerveModulePosition getPosition() {
//         return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
//     }

//     public SwerveModuleState getState() {
//         return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
//     }

//     public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward feedforward) {
//         if (Math.abs(feedforward.calculate(state.speedMetersPerSecond)) < 0.1) {
//             stop();
//             return;
//         }
//         state.optimize(getState().angle);
//         /*
//          * driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); Original line
//          */
//         driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
//         turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
//         // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
//     }

//     public void stop() {
//         driveMotor.set(0);
//         turningMotor.set(0);
//     }
// }
