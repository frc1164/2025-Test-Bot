// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmScorePosition;
import frc.robot.commands.ManualLift;
import frc.robot.commands.LEDS;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

import java.time.Instant;

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
    private final LEDSubsystem ledSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final SendableChooser<Command> autoChooser;

    private final Lift lift;
    private final Arm arm;

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Create Subsystems
        swerveSubsystem = new SwerveSubsystem();
        ledSubsystem = new LEDSubsystem();
        lift = new Lift();
        arm = new Arm();

        // Bind buttons to commands/methods
        configureBindings();

        // Setup Default Commands
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> !driverController.rightBumper().getAsBoolean()));
        

        arm.setDefaultCommand(new ManualLift(arm, driverController));
        ledSubsystem.setDefaultCommand(new LEDS(ledSubsystem, operatorController, lift));
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    private void configureBindings() {
        // Driver A Button -> Zero Heading
        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        driverController.x().onTrue(new InstantCommand(() -> lift.setLiftGoal(.25)));
        driverController.y().onTrue(new InstantCommand(() -> lift.setLiftGoal(.65)));
        driverController.b().onTrue(new InstantCommand(() -> lift.setLiftGoal(.07)));
        
        //Actual Operator Bindings:

            //Pickup
        operatorController.leftBumper().onTrue(new ParallelCommandGroup(
             new InstantCommand(() -> lift.setLiftGoal(LiftConstants.pickupHeight)),
             new InstantCommand(() -> arm.setGoal(ArmConstants.pickupSetpoint))));

            //L2
        operatorController.a().onTrue(new ParallelCommandGroup(
             new InstantCommand(() -> lift.setLiftGoal(LiftConstants.L2Height)),
             new InstantCommand(() -> arm.setGoal(ArmConstants.Up))));

            //L3
        operatorController.b().onTrue(new ParallelCommandGroup(
             new InstantCommand(() -> lift.setLiftGoal(LiftConstants.L3Height)),
             new InstantCommand(() -> arm.setGoal(ArmConstants.Up))));

            //L4
        operatorController.y().onTrue(new ParallelCommandGroup(
             new InstantCommand(() -> lift.setLiftGoal(LiftConstants.L4Height)),
             new InstantCommand(() -> arm.setGoal(ArmConstants.Up))));

            //Score(this one is going to be weird)
        operatorController.rightBumper().onTrue(new SequentialCommandGroup(
             new ArmScorePosition(arm, lift),
             new InstantCommand(() -> lift.setLiftGoal(LiftConstants.scoreHeight))));
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
