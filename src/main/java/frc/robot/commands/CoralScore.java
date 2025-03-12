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
  private EventMarker ledsOn, ledsOff;
  private boolean left;
  private LEDSubsystem m_LedSubsystem;
  // private double happy;
  // private double poseDiff;

  public CoralScore(SwerveSubsystem m_subsystem, boolean isLeft, LEDSubsystem ledSubsystem) {
    subsystem = m_subsystem;
    left = isLeft;
    m_LedSubsystem = ledSubsystem;

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
    if (left) {
      switch (subsystem.getPrincipalTag()) {
        case 17:
          targetPose = new Pose2d(3.965, 2.95, new Rotation2d(Math.PI / 3));
          break;
        case 18:
          targetPose = new Pose2d(3.165, 3.93, new Rotation2d(0));
          break;
        case 19:
          targetPose = new Pose2d(2.325, 3.72, new Rotation2d(Math.PI * 5 / 3));
          break;
        case 20:
          targetPose = new Pose2d(5.01, 5.265, new Rotation2d(Math.PI * 4 / 3));
          break;
        case 21:
          targetPose = new Pose2d(5.825, 4.16, new Rotation2d(Math.PI));
          break;
        case 22:
          targetPose = new Pose2d(5.29, 2.95, new Rotation2d(Math.PI * 2 / 3));
          break;
        default:
          targetPose = null;
          break;
      }
    } else if (!left) {
      switch (subsystem.getPrincipalTag()) {
        case 17:
          targetPose = new Pose2d(3.93, 2.82, new Rotation2d(Math.PI / 3));
          break;
        case 18:
          targetPose = new Pose2d(3.22, 3.63, new Rotation2d(0));
          break;
        case 19:
          targetPose = new Pose2d(3.66, 5.085, new Rotation2d(Math.PI * 5 / 3));
          break;
        case 20:
          targetPose = new Pose2d(5.28, 5.12, new Rotation2d(Math.PI * 4 / 3));
          break;
        case 21:
          targetPose = new Pose2d(5.825, 3.86, new Rotation2d(Math.PI));
          break;
        case 22:
          targetPose = new Pose2d(5, 2.78, new Rotation2d(Math.PI * 2 / 3));
          break;
        default:
          targetPose = null;
          break;
      }
    }
    currentPose = subsystem.getPose();
    if (targetPose != null) {
      if (!targetPose.equals(currentPose)) {
        waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
        constraints = new PathConstraints(3, 1.5, 1, .5);
        endstate = new GoalEndState(0, targetPose.getRotation());

        ledsOn = new EventMarker("leds", 0);
        ledsOn.command().alongWith(
          new InstantCommand(() -> m_LedSubsystem.setPattern3(m_LedSubsystem.colorWhite())),
          new InstantCommand(() -> m_LedSubsystem.setPattern4(m_LedSubsystem.colorWhite())));

        ledsOff = new EventMarker("leds", 1);
        ledsOff.command().alongWith(
          new InstantCommand(() -> m_LedSubsystem.setPattern3(m_LedSubsystem.colorPurple())),
          new InstantCommand(() -> m_LedSubsystem.setPattern4(m_LedSubsystem.colorOrange())));  

        path = new PathPlannerPath(waypoints, constraints, null, endstate);
      
        subsystem.currentPath = AutoBuilder.followPath(path);
        subsystem.currentPath.schedule();
      }
    }
    SmartDashboard.putNumber("seen", subsystem.getPrincipalTag());
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
