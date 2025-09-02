// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CoralScore extends Command {
  private final SwerveSubsystem subsystem;
  private List<Waypoint> waypoints;
  private PathConstraints constraints;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private PathPlannerPath path;
  private GoalEndState endstate;
  //private EventMarker ledsOn, ledsOff;
  private boolean left;
  private LEDSubsystem m_LedSubsystem;
  private int principalTag;
  private boolean flipPath;
  // private double happy;
  // private double poseDiff;

  public CoralScore(SwerveSubsystem m_subsystem, boolean isLeft, LEDSubsystem ledSubsystem) {
    subsystem = m_subsystem;
    left = isLeft;
    m_LedSubsystem = ledSubsystem;
    flipPath = false; // Default to blue side, unless changed. Prevent crashes associated with null.

  }

  @Override
  public void initialize() {
    if (subsystem.currentPath != null) {
      CommandScheduler.getInstance().cancel(subsystem.currentPath);
    }
    // AB = 18
    // CD = 17
    // EF = 22
    // GH = 21
    // IJ = 20
    // KL = 19

    principalTag = subsystem.getPrincipalTag();

      // Left side coordinates.
      if (left) {
        // Translate all tags to coordinates. Flipped below if needed.
        switch (principalTag) {
          case 17:
            // targetPose = new Pose2d(3.92, 2.806, new Rotation2d(Math.PI / 3));
            targetPose = new Pose2d(4.061, 2.61, new Rotation2d(Math.PI / 3));
            break;
          case 18:
            // targetPose = new Pose2d(3.13, 3.9, new Rotation2d(0));
            targetPose = new Pose2d(3.05, 3.69, new Rotation2d(0));
            break;
          case 19:
            // targetPose = new Pose2d(3.74, 5.182, new Rotation2d(Math.PI * 5 / 3));
            targetPose = new Pose2d(3.733, 5.24, new Rotation2d(Math.PI * 5 / 3));
            break;
          case 20:
            //targetPose = new Pose2d(4.777, 5.429, new Rotation2d(Math.PI * 4 / 3));
            targetPose = new Pose2d( 4.91, 5.436, new Rotation2d(Math.PI * 4 / 3));
            break;
          case 21:
            targetPose = new Pose2d(5.94, 4.36, new Rotation2d(Math.PI));
            break;
          case 22:
            // targetPose = new Pose2d(5.596, 3.069, new Rotation2d(Math.PI * 2 / 3));
            targetPose = new Pose2d(5.496, 2.95, new Rotation2d(Math.PI * 2 / 3));
            break;
          case 6: 
            // targetPose = new Pose2d(13.864, 2.694, new Rotation2d(Math.PI * 2 / 3));
            targetPose = new Pose2d(13.884, 2.804, new Rotation2d(Math.PI * 2 / 3));
            break;
          case 7: 
            // targetPose = new Pose2d(14.391, 4.161, new Rotation2d(Math.PI));
            targetPose = new Pose2d(14.494, 4.07, new Rotation2d(Math.PI));
            break;
          case 8: 
            // targetPose = new Pose2d(13.621, 5.263, new Rotation2d(Math.PI * 4 / 3));
            targetPose = new Pose2d(13.738, 5.291, new Rotation2d(Math.PI * 4 / 3));
            break;
          case 9:
            // targetPose = new Pose2d(12.002, 4.971, new Rotation2d(Math.PI * 5 / 3));
            targetPose = new Pose2d(11.623, 3.691, new Rotation2d(Math.PI * 5 / 3));
            break;
          case 10:
            // targetPose = new Pose2d(11.690, 3.557, new Rotation2d(0));
            targetPose = new Pose2d(11.62, 3.69, new Rotation2d(0));
            break;
          case 11:
            // targetPose = new Pose2d(12.812, 2.601, new Rotation2d(Math.PI * 1 / 3));
            targetPose = new Pose2d(12.379, 2.7606, new Rotation2d(Math.PI * 1 / 3));
            break;
          default:
            targetPose = null;
            break;
        }
      } else if (!left) { // Right side coordinates.
        switch (principalTag) {
          case 17:
            // targetPose = new Pose2d(3.93, 2.82, new Rotation2d(Math.PI / 3));
            targetPose = new Pose2d(3.81, 2.7606, new Rotation2d(Math.PI / 3));
            break;
          case 18:
            // targetPose = new Pose2d(3.13, 3.586, new Rotation2d(0));
            targetPose = new Pose2d(3.05, 3.98, new Rotation2d(0));
            break;
          case 19:
            // targetPose = new Pose2d(3.422, 4.971, new Rotation2d(Math.PI * 5 / 3));
            targetPose = new Pose2d(3.733, 5.247, new Rotation2d(Math.PI * 5 / 3));
            break;
          case 20:
            // targetPose = new Pose2d(5.041, 5.263, new Rotation2d(Math.PI * 4 / 3));
            targetPose = new Pose2d(5.169, 5.291, new Rotation2d(Math.PI * 4 / 3));
            break;
          case 21:
            targetPose = new Pose2d(5.91, 4.07, new Rotation2d(Math.PI));
            break;
          case 22:
            // targetPose = new Pose2d(5.304, 2.933, new Rotation2d(Math.PI * 2 / 3));
            targetPose = new Pose2d(5.245, 2.804, new Rotation2d(Math.PI * 2 / 3));
            break;
          case 6: 
            // targetPose = new Pose2d(14.147, 3.069, new Rotation2d(Math.PI * 2 / 3));
            targetPose = new Pose2d(13.815, 2.804, new Rotation2d(Math.PI * 2 / 3));
            break;
          case 7: 
            // targetPose = new Pose2d(14.420, 4.464, new Rotation2d(Math.PI));
            targetPose = new Pose2d(14.492, 4.36, new Rotation2d(Math.PI));
            break;
          case 8: 
            // targetPose = new Pose2d(13.328, 5.439, new Rotation2d(Math.PI * 4 / 3));
            targetPose = new Pose2d(13.487, 5.436, new Rotation2d(Math.PI * 4 / 3));
            break;
          case 9:
            // targetPose = new Pose2d(12.821, 5.487, new Rotation2d(Math.PI * 5 / 3));
            targetPose = new Pose2d(12.307, 5.247, new Rotation2d(Math.PI * 5 / 3));
            break;
          case 10:
            // targetPose = new Pose2d(11.690, 3.888, new Rotation2d(0));
            targetPose = new Pose2d(11.623, 3.9818, new Rotation2d(0));
            break;
          case 11:
            // targetPose = new Pose2d(12.509, 2.757, new Rotation2d(Math.PI * 1 / 3));
            targetPose = new Pose2d(12.630, 2.616, new Rotation2d(Math.PI * 1 / 3));

            break;
          default:
            targetPose = null;
            break;
        }
      }
      currentPose = subsystem.getPose();
      if(flipPath) {
        // If the path needs to be flipped, flip the current pose, then flip it again, so it stays in the same place.
        currentPose = FlippingUtil.flipFieldPose(currentPose);
      }
      if (targetPose != null) {
        if (!targetPose.equals(currentPose)) {
          waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
          // constraints = new PathConstraints(1, .5, 1, .5);
          constraints = new PathConstraints(1.5, 1, 1, .5);

          endstate = new GoalEndState(0, targetPose.getRotation());

          // ledsOn = new EventMarker("leds", 0);
          // ledsOn.command().alongWith(
          // new InstantCommand(() ->
          // m_LedSubsystem.setPattern3(m_LedSubsystem.colorWhite())),
          // new InstantCommand(() ->
          // m_LedSubsystem.setPattern4(m_LedSubsystem.colorWhite())));

          // ledsOff = new EventMarker("leds", 1);
          // ledsOff.command().alongWith(
          // new InstantCommand(() ->
          // m_LedSubsystem.setPattern3(m_LedSubsystem.colorPurple())),
          // new InstantCommand(() ->
          // m_LedSubsystem.setPattern4(m_LedSubsystem.colorOrange())));

          path = new PathPlannerPath(waypoints, constraints, null, endstate);
          path.preventFlipping = flipPath; // Flip path if the original tag was on the RED alliance side.
          subsystem.currentPath = AutoBuilder.followPath(path);
          subsystem.currentPath.schedule();
        }
      }
    
    SmartDashboard.putNumber("seen", principalTag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // poseDiff = Math.sqrt(Math.pow(subsystem.getPose().getX() - targetPose.getX(),
    // 2) + Math.pow(currentPose.getY() - targetPose.getY(), 2));
    // SmartDashboard.putNumber("DistToTarget", poseDiff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("CoralScore finished.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

// ORIGINAL FILE BEFORE WEIRD LOGIC WAS INTRODUCED.
/*
 * // Copyright (c) FIRST and other WPILib contributors.
 * // Open Source Software; you can modify and/or share it under the terms of
 * // the WPILib BSD license file in the root directory of this project.
 * 
 * package frc.robot.commands;
 * 
 * import java.util.List;
 * 
 * import com.pathplanner.lib.auto.AutoBuilder;
 * import com.pathplanner.lib.path.EventMarker;
 * import com.pathplanner.lib.path.GoalEndState;
 * import com.pathplanner.lib.path.PathConstraints;
 * import com.pathplanner.lib.path.PathPlannerPath;
 * import com.pathplanner.lib.path.Waypoint;
 * 
 * import edu.wpi.first.math.geometry.Pose2d;
 * import edu.wpi.first.math.geometry.Rotation2d;
 * import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 * import edu.wpi.first.wpilibj2.command.Command;
 * import edu.wpi.first.wpilibj2.command.CommandScheduler;
 * import edu.wpi.first.wpilibj2.command.InstantCommand;
 * import frc.robot.subsystems.LEDSubsystem;
 * import frc.robot.subsystems.SwerveSubsystem;
 * 
 * public class CoralScore extends Command {
 * private final SwerveSubsystem subsystem;
 * private List<Waypoint> waypoints;
 * private PathConstraints constraints;
 * private Pose2d targetPose;
 * private Pose2d currentPose;
 * private PathPlannerPath path;
 * private GoalEndState endstate;
 * private EventMarker ledsOn, ledsOff;
 * private boolean left;
 * private LEDSubsystem m_LedSubsystem;
 * // private double happy;
 * // private double poseDiff;
 * 
 * public CoralScore(SwerveSubsystem m_subsystem, boolean isLeft, LEDSubsystem
 * ledSubsystem) {
 * subsystem = m_subsystem;
 * left = isLeft;
 * m_LedSubsystem = ledSubsystem;
 * 
 * }
 * 
 * @Override
 * public void initialize() {
 * if (subsystem.currentPath != null) {
 * CommandScheduler.getInstance().cancel(subsystem.currentPath);
 * }
 * // AB = 18
 * // CD = 17
 * // EF = 22
 * // GH = 21
 * // IJ = 20
 * // KL = 19
 * if (left) {
 * switch (subsystem.getPrincipalTag()) {
 * case 17:
 * targetPose = new Pose2d(3.965, 2.95, new Rotation2d(Math.PI / 3));
 * break;
 * case 18:
 * targetPose = new Pose2d(3.165, 3.93, new Rotation2d(0));
 * break;
 * case 19:
 * targetPose = new Pose2d(2.325, 3.72, new Rotation2d(Math.PI * 5 / 3));
 * break;
 * case 20:
 * targetPose = new Pose2d(5.01, 5.265, new Rotation2d(Math.PI * 4 / 3));
 * break;
 * case 21:
 * targetPose = new Pose2d(5.825, 4.16, new Rotation2d(Math.PI));
 * break;
 * case 22:
 * targetPose = new Pose2d(5.29, 2.95, new Rotation2d(Math.PI * 2 / 3));
 * break;
 * default:
 * targetPose = null;
 * break;
 * }
 * } else if (!left) {
 * switch (subsystem.getPrincipalTag()) {
 * case 17:
 * targetPose = new Pose2d(3.93, 2.82, new Rotation2d(Math.PI / 3));
 * break;
 * case 18:
 * targetPose = new Pose2d(3.22, 3.63, new Rotation2d(0));
 * break;
 * case 19:
 * targetPose = new Pose2d(3.66, 5.085, new Rotation2d(Math.PI * 5 / 3));
 * break;
 * case 20:
 * targetPose = new Pose2d(5.28, 5.12, new Rotation2d(Math.PI * 4 / 3));
 * break;
 * case 21:
 * targetPose = new Pose2d(5.825, 3.86, new Rotation2d(Math.PI));
 * break;
 * case 22:
 * targetPose = new Pose2d(5, 2.78, new Rotation2d(Math.PI * 2 / 3));
 * break;
 * default:
 * targetPose = null;
 * break;
 * }
 * }
 * currentPose = subsystem.getPose();
 * if (targetPose != null) {
 * if (!targetPose.equals(currentPose)) {
 * waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
 * constraints = new PathConstraints(3, 1.5, 1, .5);
 * endstate = new GoalEndState(0, targetPose.getRotation());
 * 
 * // ledsOn = new EventMarker("leds", 0);
 * // ledsOn.command().alongWith(
 * // new InstantCommand(() ->
 * m_LedSubsystem.setPattern3(m_LedSubsystem.colorWhite())),
 * // new InstantCommand(() ->
 * m_LedSubsystem.setPattern4(m_LedSubsystem.colorWhite())));
 * 
 * // ledsOff = new EventMarker("leds", 1);
 * // ledsOff.command().alongWith(
 * // new InstantCommand(() ->
 * m_LedSubsystem.setPattern3(m_LedSubsystem.colorPurple())),
 * // new InstantCommand(() ->
 * m_LedSubsystem.setPattern4(m_LedSubsystem.colorOrange())));
 * 
 * path = new PathPlannerPath(waypoints, constraints, null, endstate);
 * 
 * subsystem.currentPath = AutoBuilder.followPath(path);
 * subsystem.currentPath.schedule();
 * }
 * }
 * SmartDashboard.putNumber("seen", subsystem.getPrincipalTag());
 * }
 * 
 * // Called every time the scheduler runs while the command is scheduled.
 * 
 * @Override
 * public void execute() {
 * // poseDiff = Math.sqrt(Math.pow(subsystem.getPose().getX() -
 * targetPose.getX(),
 * // 2) + Math.pow(currentPose.getY() - targetPose.getY(), 2));
 * // SmartDashboard.putNumber("DistToTarget", poseDiff);
 * }
 * 
 * // Called once the command ends or is interrupted.
 * 
 * @Override
 * public void end(boolean interrupted) {
 * }
 * 
 * // Returns true when the command should end.
 * 
 * @Override
 * public boolean isFinished() {
 * return true;
 * }
 * }
 * 
 */