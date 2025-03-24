// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmScorePosition extends Command {
  private Arm arm;
  private Lift lift;
  /** Creates a new ArmScorePosition. */
  public ArmScorePosition(Arm m_arm, Lift m_lift) {
    lift = m_lift;
    arm = m_arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch ((int)(lift.getGoal() * 1000)){
      case (int)(LiftConstants.L4Height * 1000): arm.setGoal(ArmConstants.L4);
        break;
      case (int)(LiftConstants.L3Height * 1000): arm.setGoal(ArmConstants.L3);
        break;
      case (int)(LiftConstants.L2Height * 1000): arm.setGoal(ArmConstants.L2);
        break;
      default: arm.setGoal(arm.getPosition());
    }
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
    return arm.atSetpoint();
  }
}
