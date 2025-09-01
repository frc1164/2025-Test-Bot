package frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.BuildConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;
import frc.robot.util.LocalADStarAK;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveSubsystem extends SubsystemBase {
    private final ModuleIO frontLeftIO;
    private final ModuleIO frontRightIO;
    private final ModuleIO backLeftIO;
    private final ModuleIO backRightIO;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

      private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);


    public Command currentPath;
    private final Pose2d poseThis = new Pose2d();
    private SwerveModulePosition[] Position = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            new Rotation2d(0), Position, poseThis);

    // Create a new Field2d object for plotting pose and initialize LimeLight
    // Network table instances
    private final Field2d m_field = new Field2d();

    // Limelight Definitions
    private final NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.kLLTags);
    private double tv, ta, tl;
    private boolean isUpdating = false;
    private boolean gate = true;
    private boolean updatingSet = false;

    private boolean isUpdatingSet = false;
    private boolean canSeeTagsSet = true;
    private boolean elseSet = false;

    // private LimelightHelpers.LimelightResults results;
    private LimelightHelpers.PoseEstimate limelightMeasurement;

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    // Create two new SimpleMotorFeedforwards (one right and one left) with gains
    // kS, kV, and kA from SysID characterization
    private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(DriveConstants.kSRight,
            DriveConstants.kVRight, DriveConstants.kARight);
    private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(DriveConstants.kSLeft,
            DriveConstants.kVLeft, DriveConstants.kALeft);

    private double tag;
    private int tagRead;

    // AdvantageKit
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    static final double ODOMETRY_FREQUENCY = new CANBus().isNetworkFD() ? 250.0 : 100.0;

    public SwerveSubsystem(ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO, GyroIO gyroIO) {
        this.frontLeftIO = flModuleIO;
        this.frontRightIO = frModuleIO;
        this.backLeftIO = blModuleIO;
        this.backRightIO = brModuleIO;
        this.gyroIO = gyroIO;

        this.frontLeft = new SwerveModule(flModuleIO, 1);
        this.frontRight = new SwerveModule(frModuleIO, 2);
        this.backLeft = new SwerveModule(blModuleIO, 3);
        this.backRight = new SwerveModule(brModuleIO, 4);

        PhoenixOdometryThread.getInstance().start();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        try {
            // This WILL FAIL if the file (/src/main/deploy/pathplanner/settings.json) is
            // not present.
            // Make sure to open PathPlanner and change a setting to create the file.
            RobotConfig config = RobotConfig.fromGUISettings();
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforward) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                         // RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                    // Constants class
                            new PIDConstants(AutoConstants.kPTranslationController, 0.0,
                                    AutoConstants.kDTranslationController), // Translation PID constants
                            new PIDConstants(AutoConstants.kPThetaController, 0.0, AutoConstants.kDThetaController) // Rotation
                                                                                                                    // PID
                                                                                                                    // constants
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
            Pathfinding.setPathfinder(new LocalADStarAK());
            PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                  Logger.recordOutput(
                      "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
            PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                  Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

                
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder. Ensure /src/main/deploy/pathplanner/settings.json exists",
                    e.getStackTrace());
        }
        setCurrentGyroHeading(180);
    }

    public void zeroHeading() {
        gyroIO.resetHeading();
    }

    private void setCurrentGyroHeading(double heading) {
        gyroIO.setAngleAdjustment(heading);
    }

    public Rotation2d getHeading() {
        return gyroInputs.yawPosition;
    }
    
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] state = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        m_poseEstimator.resetPosition(getHeading(), state, pose);
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
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

    public LimelightHelpers.PoseEstimate getVisionEstimatedPose() {

        LimelightHelpers.SetRobotOrientation("limelight-tags", getHeading().getDegrees(), getYawRate(), 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-tags");

        // double[] bot_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // double bot_x, bot_y, rotation_z;

        // bot_pose = aprilTagTable
        // .getEntry("botpose_orb_wpiblue")
        // .getDoubleArray(new double[6]);

        return botPose;
    }

    public void updatePoseEstimatorWithVisionBotPose(LimelightHelpers.PoseEstimate poseEstimate) {
        // PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
        // Pose2d visionPose = getVisionEstimatedPose();
        Pose2d visionPose = poseEstimate.pose;
        // invalid LL data
        // if (visionBotPose.pose2d.getX() == 0.0) {
        // return;
        // }

        if (visionPose.getX() == 0.0) {
            isUpdating = false;
            return;
        }

        // distance from current pose to vision estimated pose
        // double poseDifference =
        // m_poseEstimator.getEstimatedPosition().getTranslation()
        // .getDistance(visionBotPose.pose2d.getTranslation());

        double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
                .getDistance(visionPose.getTranslation());

        if (poseEstimate.tagCount > 0) {
            double xyStds;
            double degStds;
            SmartDashboard.putNumber("poseDifference", poseDifference);
            // multiple targets detected
            if (poseEstimate.tagCount >= 2 && poseEstimate.avgTagArea > 0.5) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.66 && poseDifference < 1.5) { // areea 0.8, diff 0.5
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.15 && poseDifference < 0.3) { // area 0.1, diff 0.3
                xyStds = 2.0;
                degStds = 30;
            } else if (gate) {
                xyStds = 0;
                degStds = 0;
                gate = false;
            }
            // conditions don't match to add a vision measurement
            else {
                isUpdating = false;
                return;
            }

            isUpdating = true;

            m_poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            m_poseEstimator.addVisionMeasurement(visionPose,
                    poseEstimate.timestampSeconds);
        }
    }

    public double getLatency() {
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl);

        // maybe need camera_latency?
        // TODO: TEST
        // return results.targetingResults.latency_capture;

        // TODO: TEST this breaks it for some reason
        // return llresults.targetingResults.latency_pipeline;
    }

    public int getPrincipalTag() {
        tag = aprilTagTable.getValue("tid").getDouble();
        if (tag == 0) {
        } else {
            tagRead = (int) tag;
        }

        return tagRead;
    }

    public double getYawRate() {
        return gyroInputs.yawRate;
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            stopModules();

            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        SwerveModulePosition[] positions = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        m_poseEstimator.update(gyroInputs.yawPosition, positions);

        // The .name() method seems to have been removed from DriverStation.getAlliance.
        // So this needs a switch statement or something
        // SmartDashboard.putString("Alliance Color",
        // DriverStation.getAlliance().name());

        // Set the robot pose on the Field2D object

        m_field.setRobotPose(this.getPose());
        SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());

        SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("Pitch", gyroInputs.pitchPosition.getDegrees());
        SmartDashboard.putNumber("Yaw", gyroInputs.yawPosition.getDegrees());
        SmartDashboard.putNumber("Roll", gyroInputs.rollPosition.getDegrees());

        // LimelightHelpers.PoseEstimate tagsLLPoseEstimate =
        // LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLightConstants.kLLTags);

        boolean signalIsUpdating = false;

        // updatePoseEstimatorWithVisionBotPose(tagsLLPoseEstimate);
        // if(isUpdating == true) {
        // signalIsUpdating = true;

        SmartDashboard.putNumber("ta", aprilTagTable.getValue("ta").getDouble());

        updatePoseEstimatorWithVisionBotPose(getVisionEstimatedPose());
        if (isUpdating == true) {
            signalIsUpdating = true;
        }
        SmartDashboard.putBoolean("signalIsUpdating", signalIsUpdating);
        SmartDashboard.putBoolean("seesTags", getVisionEstimatedPose().tagCount > 0);

        gyroDisconnectedAlert.set(!gyroInputs.connected && AdvantageKitConstants.currentMode != AdvantageKitConstants.Mode.SIM);
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

    public Rotation2d getChassisYaw() {
        return gyroInputs.yawPosition;
    }
}
