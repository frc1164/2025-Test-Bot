// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AdvantageKitConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmScorePosition;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.RunClimb;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.CoralScore;
import frc.robot.commands.IntakeCheck;
import frc.robot.commands.LiftSetpoint;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveSubsystem.GyroIO;
import frc.robot.subsystems.SwerveSubsystem.GyroIONavX;
import frc.robot.subsystems.SwerveSubsystem.ModuleIO;
import frc.robot.subsystems.SwerveSubsystem.ModuleIOSim;
import frc.robot.subsystems.SwerveSubsystem.ModuleIOTalonFX;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final LEDSubsystem ledSubsystem;
    private final SendableChooser<Command> autoChooser;

    private final SwerveSubsystem swerveSubsystem;
    // private final Lift lift;
    // private final Arm arm;
    private final Climb climb;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Create Subsystems
        switch (AdvantageKitConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveSubsystem = new SwerveSubsystem(
                        new ModuleIOTalonFX(DriveConstants.kFrontLeftDriveMotorPort,
                                DriveConstants.kFrontLeftTurningMotorPort,
                                DriveConstants.kFrontLeftDriveEncoderReversed,
                                DriveConstants.kFrontLeftTurningEncoderReversed,
                                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                                DriveConstants.kSLeft,
                                DriveConstants.kVLeft,
                                DriveConstants.kALeft),
                        new ModuleIOTalonFX(DriveConstants.kFrontRightDriveMotorPort,
                                DriveConstants.kFrontRightTurningMotorPort,
                                DriveConstants.kFrontRightDriveEncoderReversed,
                                DriveConstants.kFrontRightTurningEncoderReversed,
                                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                                DriveConstants.kSRight,
                                DriveConstants.kVRight,
                                DriveConstants.kARight),
                        new ModuleIOTalonFX(DriveConstants.kBackLeftDriveMotorPort,
                                DriveConstants.kBackLeftTurningMotorPort,
                                DriveConstants.kBackLeftDriveEncoderReversed,
                                DriveConstants.kBackLeftTurningEncoderReversed,
                                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                                DriveConstants.kSLeft,
                                DriveConstants.kVLeft,
                                DriveConstants.kALeft),
                        new ModuleIOTalonFX(DriveConstants.kBackRightDriveMotorPort,
                                DriveConstants.kBackRightTurningMotorPort,
                                DriveConstants.kBackRightDriveEncoderReversed,
                                DriveConstants.kBackRightTurningEncoderReversed,
                                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                                DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                                DriveConstants.kSRight,
                                DriveConstants.kVRight,
                                DriveConstants.kARight),
                        new GyroIONavX());
                break;

            // case SIM:
                // Sim robot, instantiate physics sim IO implementations
                // swerveSubsystem = new SwerveSubsystem(
                // new ModuleIOSim(),
                // new ModuleIOSim(),
                // new ModuleIOSim(),
                // new ModuleIOSim(),
                // new GyroIO() {
                // });
                // break;

            default:
                // Replayed robot, disable IO implementations
                swerveSubsystem = new SwerveSubsystem(
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new GyroIO() {
                        });

                break;
        }
        // lift = new Lift();
        // arm = new Arm();
        // ledSubsystem = new LEDSubsystem(lift, arm);
        climb = new Climb();

        // Bind buttons to commands/methods
        configureBindings();

        // Add PathPlanner Named Commands
        // NamedCommands.registerCommand("L4", new SequentialCommandGroup(
        // new LiftSetpoint(lift, LiftConstants.L4Height),
        // new ArmSetpoint(arm, ArmConstants.UpL4),
        // new InstantCommand(() -> {
        // System.out.println("L4 Finished");
        // })));

        // NamedCommands.registerCommand("L3", new SequentialCommandGroup(
        // new LiftSetpoint(lift, LiftConstants.L3Height),
        // new ArmSetpoint(arm, ArmConstants.Up)));

        // NamedCommands.registerCommand("L2", new SequentialCommandGroup(
        // new LiftSetpoint(lift, LiftConstants.L2Height),
        // new ArmSetpoint(arm, ArmConstants.Up)));

        // NamedCommands.registerCommand("L4Lift", new LiftSetpoint(lift,
        // LiftConstants.L4Height));

        // NamedCommands.registerCommand("L4Arm", new InstantCommand(() -> {
        // arm.setGoal(ArmConstants.UpL4);
        // }));

        // NamedCommands.registerCommand("IntakeArm", new ArmSetpoint(arm,
        // ArmConstants.pickupSetpoint));

        // NamedCommands.registerCommand("Arm Score", new ArmScorePosition(arm, lift));

        // NamedCommands.registerCommand("IntakeLift", new LiftSetpoint(lift,
        // LiftConstants.pickupHeight));

        // NamedCommands.registerCommand("ScoreLift", new LiftSetpoint(lift,
        // LiftConstants.scoreHeight));

        // NamedCommands.registerCommand("IntakeLight",
        // new InstantCommand(() ->
        // ledSubsystem.setPattern3(ledSubsystem.scrollingRainbow())));

        // NamedCommands.registerCommand("IntakeCheck", new IntakeCheck(arm));

        // NamedCommands.registerCommand("L3 Height", new LiftSetpoint(lift,
        // LiftConstants.L3Height));

        // NamedCommands.registerCommand("AlignL", new CoralScore(swerveSubsystem, true,
        // ledSubsystem));
        // NamedCommands.registerCommand("AlignR", new CoralScore(swerveSubsystem,
        // false, ledSubsystem));

        // Setup Default Commands
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> !driverController.rightBumper().getAsBoolean()));

        // climb.setDefaultCommand(new RunClimb(climb, operatorController));

        // // Build an auto chooser. This will use Commands.none() as the default
        // option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureBindings() {
        // Driver A Button -> Zero Heading
        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.setPose(
                new Pose2d(swerveSubsystem.getPose().getTranslation(), Rotation2d.kZero))));
        // driverController.povUp().onTrue(new SequentialCommandGroup(
        // new InstantCommand(() ->
        // ledSubsystem.setPattern3(ledSubsystem.scrollingRainbow())),
        // new InstantCommand(() ->
        // ledSubsystem.setPattern4(ledSubsystem.scrollingRainbow()))));

        // Actual Operator Bindings:

        // Pickup
        // operatorController.leftBumper().onTrue(new SequentialCommandGroup(
        // new ArmSetpoint(arm, ArmConstants.pickupSetpoint),
        // new LiftSetpoint(lift, LiftConstants.pickupHeight)));

        // // L2
        // operatorController.a().onTrue(new SequentialCommandGroup(
        // new LiftSetpoint(lift, LiftConstants.L2Height),
        // new ArmSetpoint(arm, 4.5)));
        // // new InstantCommand(() -> arm.setGoal(ArmConstants.Up))));

        // // L3
        // operatorController.b().onTrue(new SequentialCommandGroup(
        // new LiftSetpoint(lift, LiftConstants.L3Height),
        // // new ArmSetpoint(arm, ArmConstants.Up)));
        // new ArmSetpoint(arm, 4.5)));

        // // L4
        // operatorController.y().onTrue(new SequentialCommandGroup(
        // new LiftSetpoint(lift, LiftConstants.L4Height),
        // new ArmSetpoint(arm, ArmConstants.UpL4)));

        // // new InstantCommand(() -> arm.setGoal(ArmConstants.Up))));

        // // Score(this one is going to be weird)
        // operatorController.rightBumper().onTrue(new SequentialCommandGroup(
        // new ArmScorePosition(arm, lift),
        // new LiftSetpoint(lift, LiftConstants.scoreHeight)));

        // driverController.rightBumper().onTrue(new CoralScore(swerveSubsystem, false,
        // ledSubsystem));

        // driverController.leftBumper().onTrue(new CoralScore(swerveSubsystem, true,
        // ledSubsystem));

        // Down on POV as gate operator. This may be wrong. Check DS to double check.
        // Pressing down allows the RunClimb() command to work.
        operatorController.povDown().onTrue(new InstantCommand(() -> {
            climb.runGate = true;
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
