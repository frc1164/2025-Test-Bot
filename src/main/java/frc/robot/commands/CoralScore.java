// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class CoralScore extends Command {
  private final SwerveSubsystem subsystem;
  private List <Waypoint> waypoints;
  private PathConstraints constraints;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private PathPlannerPath path;
  private GoalEndState endstate;
  private boolean left;
  //private double happy;
  //private double poseDiff;

  public CoralScore(SwerveSubsystem m_subsystem, boolean isLeft) {
    subsystem = m_subsystem;
    left = isLeft;

  }
      
  @Override
  public void initialize() {
    currentPose = subsystem.getPose();
    

    //AB = 18
    //CD = 17
    //EF = 22
    //GH = 21
    //IJ = 20
    //KL = 19
    if(left){
      switch(subsystem.getPrincipalTag()){
          case 17: targetPose = new Pose2d(3.965, 2.95, new Rotation2d(Math.PI/3));
        break;
          case 18: targetPose = new Pose2d(3.165, 4.165, new Rotation2d(0));
        break; 
          case 19: targetPose = new Pose2d(2.325, 3.72, new Rotation2d(Math.PI * 5/3));
        break;
          case 20: targetPose = new Pose2d(5.01, 5.265, new Rotation2d(Math.PI * 4/3));
        break;
          case 21: targetPose = new Pose2d(5.825, 4.16, new Rotation2d(Math.PI));
        break;
          case 22: targetPose = new Pose2d(5.29, 2.95, new Rotation2d(Math.PI * 2/3));
        break;
      }
    }else if (!left){
      switch (subsystem.getPrincipalTag()){
          case 17: targetPose = new Pose2d(3.93, 2.82, new Rotation2d(Math.PI/3));
        break;
          case 18: targetPose = new Pose2d(3.165, 3.875, new Rotation2d(0));
        break;
          case 19: targetPose = new Pose2d(3.66, 5.085, new Rotation2d(Math.PI * 5/3));
        break;
          case 20: targetPose = new Pose2d(5.28, 5.12, new Rotation2d(Math.PI * 4/3));
        break;
          case 21: targetPose = new Pose2d(5.825, 3.86, new Rotation2d(Math.PI));
        break;
          case 22: targetPose = new Pose2d(5, 2.78, new Rotation2d(Math.PI * 2/3));
        break;
      }
    }
    waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);   
    constraints = new PathConstraints(3, 1.5, 1, .5);
    endstate = new GoalEndState(0, targetPose.getRotation());
    path = new PathPlannerPath(waypoints, constraints, null, endstate);
    AutoBuilder.followPath(path).schedule();

    SmartDashboard.putNumber("seen", subsystem.getPrincipalTag());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // poseDiff = Math.sqrt(Math.pow(subsystem.getPose().getX() - targetPose.getX(), 2) + Math.pow(currentPose.getY() - targetPose.getY(), 2));
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
