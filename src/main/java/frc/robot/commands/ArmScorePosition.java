// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmScorePosition extends InstantCommand {
  private Arm arm;
  private Lift lift;
  public ArmScorePosition(Arm m_arm, Lift m_lift) {
    arm = m_arm;
    lift = m_lift;
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
    }

  }
}
