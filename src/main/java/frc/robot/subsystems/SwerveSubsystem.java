package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(NavXComType.kUSB1);
    private final Pose2d poseThis = new Pose2d();
    private final SwerveModulePosition[] Position = { frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition() };
    //private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            //new Rotation2d(0), Position, poseThis);

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            new Rotation2d(0), Position, poseThis);

    // Create a new Field2d object for plotting pose and initialize LimeLight Network table instances
    private final Field2d m_field = new Field2d();
  


    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    // Create two new SimpleMotorFeedforwards (one right and one left) with gains
    // kS, kV, and kA from SysID characterization
    private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(DriveConstants.kSRight,
            DriveConstants.kVRight, DriveConstants.kARight);
    private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(DriveConstants.kSLeft,
            DriveConstants.kVLeft, DriveConstants.kALeft);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        try {
        RobotConfig config = RobotConfig.fromGUISettings();
        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforward) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
              Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
        }
        catch(Exception e){
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",e.getStackTrace());
        }
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] state = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        m_poseEstimator.resetPosition(getRotation2d(), state, pose);
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/targetStates", targetStates);
        
        setModuleStates(targetStates);
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

   

    @Override
    public void periodic() {
        // TODO: Log Gyro data
        SwerveModulePosition[] positions = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        m_poseEstimator.update(getRotation2d(), positions);

        // The .name() method seems to have been removed from DriverStation.getAlliance. So this needs a switch statement or something
        //SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());
        
        // Set the robot pose on the Field2D object
        
      
       

        m_field.setRobotPose(this.getPose());
        SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        // SmartDashboard.putNumber("Pitch Rate", gyro.getRawGyroX());
        // SmartDashboard.putNumber("Yaw Rate", gyro.getRawGyroY());
        // SmartDashboard.putNumber("Roll Rate", gyro.getRawGyroZ());
        // SmartDashboard.putNumber("X Acceleration", gyro.getWorldLinearAccelX());
        // SmartDashboard.putNumber("Y Acceleration", gyro.getWorldLinearAccelY());
        //offsets
        //forward: 0.381
        //up: 0.713

        // This is not finished yet... :-)
        // if (isUpdating && !isUpdatingSet) {
        //     m_LEDs.signal(statusLED.STRIP3, ledMode.PURPLE);
        //     isUpdatingSet = true;
        // }
        // else if (!isUpdating && !updatingSet) {
        //     m_LEDs.signal(statusLED.STRIP3, ledMode.ORANGE);
        //     updatingSet = true;
        // }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], feedforwardLeft);
        frontRight.setDesiredState(desiredStates[1], feedforwardRight);
        backLeft.setDesiredState(desiredStates[2], feedforwardLeft);
        backRight.setDesiredState(desiredStates[3], feedforwardRight);
    }

    // Though related to the LimeLights, these functions are needed for pose esimation.
    
    
    /* This can probably be safely removed
    public float getChassisPitch() {
        return gyro.getPitch();
    }*/

    public float getChassisYaw() {
        return gyro.getYaw();
    }

    } 
