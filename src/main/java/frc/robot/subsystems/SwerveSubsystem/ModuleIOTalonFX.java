package frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

// import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>
 * Device configuration and other behaviors not exposed by TunerConstants can be
 * customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
    // private final SwerveModuleConstants<TalonFXConfiguration,
    // TalonFXConfiguration, CANcoderConfiguration> constants;

    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;
    private final PIDController turnPidController;
    private final SimpleMotorFeedforward driveFeedforward;
    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFX(
            int driveMotorId, int turningMotorId, InvertedValue driveMotorReversed,
            InvertedValue turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, double kS, double kV,
            double kA) {
        // this.constants = constants;
        driveTalon = new TalonFX(driveMotorId);
        turnTalon = new TalonFX(turningMotorId);
        cancoder = new CANcoder(absoluteEncoderId);

        turnPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turnPidController.enableContinuousInput(-.5, .5);
        turnPidController.setTolerance(.003);

        driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
        // Configure drive motor
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveMotorGearRatio;
        // driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        // driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        // driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = driveMotorReversed;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoderId;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnConfig.Feedback.RotorToSensorRatio = ModuleConstants.kTurningMotorGearRatio;
        turnConfig.Feedback.SensorToMechanismRatio = 1;
        // turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 /
        // constants.SteerMotorGearRatio;
        // turnConfig.MotionMagic.MotionMagicAcceleration =
        // turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        // turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 *
        // constants.SteerMotorGearRatio;
        // turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted = turningMotorReversed;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        ;
        cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = absoluteEncoderReversed
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoder.getConfigurator().apply(cancoderConfig);
}

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Create drive status signals
        double drivePosition = driveTalon.getPosition().getValueAsDouble();
        double driveVelocity = driveTalon.getVelocity().getValueAsDouble();
        double driveAppliedVolts = driveTalon.getMotorVoltage().getValueAsDouble();
        double driveCurrent = driveTalon.getStatorCurrent().getValueAsDouble();

        // Create turn status signals
        double turnAbsolutePosition = cancoder.getAbsolutePosition().getValueAsDouble();
        double turnPosition = turnTalon.getPosition().getValueAsDouble();
        double turnVelocity = turnTalon.getVelocity().getValueAsDouble();
        double turnAppliedVolts = turnTalon.getMotorVoltage().getValueAsDouble();
        double turnCurrent = turnTalon.getStatorCurrent().getValueAsDouble();

        // Update drive inputs
        inputs.driveConnected = driveTalon.isConnected();
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition);
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = driveCurrent;

        // Update turn inputs
        inputs.turnConnected = turnTalon.isConnected();
        
        inputs.turnEncoderConnected = cancoder.isConnected();
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition);
        inputs.turnPosition = Rotation2d.fromRotations((turnPosition + 0.5) % 1 - 0.5);
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = turnCurrent;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveTalon.setVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turnTalon.setVoltage(voltage);
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
        driveTalon.setVoltage(driveFeedforward.calculate(velocityMetersPerSec));
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnTalon.setVoltage(turnPidController.calculate(cancoder.getAbsolutePosition().getValueAsDouble(), rotation.getRotations()));
    }
}