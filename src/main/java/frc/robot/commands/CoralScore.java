// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class CoralScore extends Command {
  private final SwerveSubsystem subsystem;
  private List <Waypoint> waypoints;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private AutoConstants.location location;



  public CoralScore(AutoConstants.location target, SwerveSubsystem m_subsystem) {
    subsystem = m_subsystem;
    location = target;
  }
      
  @Override
  public void initialize() {
    currentPose = subsystem.getPose();

    //Read limelight get "tid"
    //if left, get the corresponding left and same for right
    //AB = 18
    //CD = 17
    //EF = 22
    //GH = 21
    //IJ = 20
    //KL = 19
    switch (location){
      case A: targetPose = new Pose2d(3.165, 4.165, new Rotation2d(0));
        break;
      case B: targetPose = new Pose2d(3.165, 3.875, new Rotation2d(0));
        break;
      case C: targetPose = new Pose2d(3.965, 2.95, new Rotation2d(Math.PI/3));
        break;
      case D: targetPose = new Pose2d(3.93, 2.82, new Rotation2d(Math.PI/3));
        break;
      case E: targetPose = new Pose2d(5, 2.78, new Rotation2d(Math.PI * 2/3));
        break;
      case F: targetPose = new Pose2d(5.29, 2.95, new Rotation2d(Math.PI * 2/3));
        break;
      case G: targetPose = new Pose2d(5.825, 3.86, new Rotation2d(Math.PI));
        break;
      case H: targetPose = new Pose2d(5.825, 4.16, new Rotation2d(Math.PI));
        break;
      case I: targetPose = new Pose2d(5.28, 5.12, new Rotation2d(Math.PI * 4/3));
        break;
      case J: targetPose = new Pose2d(5.01, 5.265, new Rotation2d(Math.PI * 4/3));
        break;
      case K: targetPose = new Pose2d(2.325, 3.72, new Rotation2d(Math.PI * 5/3));
        break;
      case L: targetPose = new Pose2d(3.66, 5.085, new Rotation2d(Math.PI * 5/3));
    }
    waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
