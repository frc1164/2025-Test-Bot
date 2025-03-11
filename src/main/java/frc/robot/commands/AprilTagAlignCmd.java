// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagAlignCmd extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final PIDController lateralPidController;
  private final PIDController yawPidController;
  private final NetworkTable limelighNetworkTable;
  private final double offset;
  private double tv, tx, ty, ta;

  /** Creates a new AprilTagAlignCmd. */
  public AprilTagAlignCmd(SwerveSubsystem swerveSubsystem, double offset) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.lateralPidController = new PIDController(0.5, 0.015, .001);
    yawPidController = new PIDController(0.25, 0, 0);
    this.limelighNetworkTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.kTagLimelightNetworkTableName);
    
    this.offset = offset;
    lateralPidController.setSetpoint(0);
    yawPidController.setSetpoint(0);
    addRequirements(this.m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelighNetworkTable.getEntry("pipeline").setNumber(LimeLightConstants.kAprilTagPipeline);
    SmartDashboard.putBoolean("AprilTagAlign", true);
    lateralPidController.setTolerance(.05);
    yawPidController.setTolerance(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lastTx = tx;
    ChassisSpeeds chassisSpeeds;
    double [] poseArray = limelighNetworkTable.getEntry("targetpose_robotspace").getDoubleArray(new double [6]);
    tx = poseArray[0];
    tv = limelighNetworkTable.getEntry("tv").getDouble(0);

    if (tv == 1){
      tx = tx; 
    }else{
      tx = lastTx;
    }

    ty = limelighNetworkTable.getEntry("ty").getDouble(0);
    ta = limelighNetworkTable.getEntry("ta").getDouble(0);

    double tyaw = poseArray[4] * Math.PI/180;
    SmartDashboard.putNumber("align-tx", tx);
    SmartDashboard.putNumber("align-yaw", poseArray[4]);

    
    chassisSpeeds = new ChassisSpeeds(0, lateralPidController.calculate(tx), yawPidController.calculate(tyaw));

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Output each module states to wheels
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AprilTagAlign", false);
    limelighNetworkTable.getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if(yawPidController.atSetpoint() && lateralPidController.atSetpoint()){
    //   return true;
    // }else{
    //   return false;
    // }
    return false;
  }
}
