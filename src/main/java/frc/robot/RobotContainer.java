// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OffsetConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignCmd;
import frc.robot.commands.SwerveJoystickCmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

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
    private final SwerveSubsystem swerveSubsystem;
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController m_driverXboxController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorXboxController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Create Subsystems
        swerveSubsystem = new SwerveSubsystem();

        // Bind buttons to commands/methods
        configureBindings();

        // Setup Default Commands
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> m_driverXboxController.getLeftY(),
                () -> m_driverXboxController.getLeftX(),
                () -> -m_driverXboxController.getRightX(),
                () -> !m_driverXboxController.rightBumper().getAsBoolean()));
        

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
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
        m_driverXboxController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        m_driverXboxController.rightBumper().whileTrue(new AprilTagAlignCmd(swerveSubsystem, OffsetConstants.offsetRight));
        m_driverXboxController.leftBumper().whileTrue(new AprilTagAlignCmd(swerveSubsystem, OffsetConstants.offsetLeft));






        m_driverXboxController.x().whileTrue(AutoBuilder.pathfindToPose(new Pose2d(7.7, 6.1, Rotation2d.fromDegrees(0)),
                                                                        new PathConstraints(.5, .25, 1, 1)));
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
