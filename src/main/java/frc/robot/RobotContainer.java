// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.LEDS;
import frc.robot.commands.AprilTagAlignCmd;
import frc.robot.commands.SwerveJoystickCmd;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.DriverStation;

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


    
        private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController m_driverXboxController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController m_operatorXboxController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Create Subsystems
        swerveSubsystem = new SwerveSubsystem();
    ledSubsystem = new LEDSubsystem();

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
        ledSubsystem.setDefaultCommand(new LEDS(ledSubsystem, swerveSubsystem));

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
       

        // try {
        //     PathPlannerPath path = PathPlannerPath.fromPathFile("Blu-Close-R-R");
        //     m_driverXboxController.x().whileTrue(AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(2, 1, 1.5, .25)));
        // } catch (Exception e) {
        //         DriverStation.reportError("oopsie daisy!!: " + e.getMessage(), e.getStackTrace());
        //     }


        // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
        //     Pose2d currentPose = swerveSubsystem.getPose();
            
        //     // The rotation component in these poses represents the direction of travel
        //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        //     Pose2d endPos = new Pose2d(3.2, 3.863, new Rotation2d());
      
        //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        //     PathPlannerPath path = new PathPlannerPath(
        //       waypoints, 
        //       new PathConstraints(.75, 1, 1.5, .25),
        //       null, // Ideal starting state can be null for on-the-fly paths
        //       new GoalEndState(0.0, new Rotation2d(0))
        //     );
      
        //     // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        //     path.preventFlipping = true;
      
        //     AutoBuilder.followPath(path).schedule();
        //     }));

        
        m_driverXboxController.rightBumper().onTrue(Commands.runOnce(() -> {makePath(new Pose2d(3.165, 3.863, new Rotation2d(0)));}));
         m_driverXboxController.leftBumper().onTrue(Commands.runOnce(() -> {makePath(new Pose2d(3.165, 4.163, new Rotation2d(0)));}));
        }

    private void makePath(Pose2d targetPose){
        final List bPoints = PathPlannerPath.waypointsFromPoses(swerveSubsystem.getPose(), targetPose);
        final PathConstraints constraints = new PathConstraints(.75, 1, 1.5, .25);

        PathPlannerPath testPath = new PathPlannerPath(bPoints,
        constraints,
        null,
        new GoalEndState(0, new Rotation2d(0)));
        AutoBuilder.followPath(testPath).schedule();
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
  }}
//}
