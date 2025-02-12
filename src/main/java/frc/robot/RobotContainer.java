// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;

import com.fasterxml.jackson.databind.jsontype.BasicPolymorphicTypeValidator.NameMatcher;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.LEDS;
import frc.robot.commands.SwerveJoystickCmd;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 private final LEDSubsystem ledSubsystem;
   private final SwerveSubsystem swerveSubsystem;

        private final SendableChooser<Command> autoChooser;

        private final CommandXboxController operatorController = new CommandXboxController(1);

       

        private final Joystick m_driveController = new Joystick(OIConstants.kDriverControllerPort);
      

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerveSubsystem = new SwerveSubsystem();
    ledSubsystem = new LEDSubsystem();

   
    configureBindings();


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> m_driveController.getRawAxis(OIConstants.kDriverYAxis),
                                () -> m_driveController.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -m_driveController.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !m_driveController.getRawButton(6))); // RB

                 //Register named commands
             
              // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    
   ledSubsystem.setDefaultCommand(new LEDS(ledSubsystem, operatorController));
    
     
    
    //ledSubsystem.setDefaultCommand(ledSubsystem.runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    
    
  //   // A Button
    new JoystickButton(m_driveController, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
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
//}
