// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftSetpoint extends Command {
  private Lift lift;
  private double setpoint;
  private Runnable nextCommandList;
  /** Creates a new LiftSetpoint. */
  public LiftSetpoint(Lift m_lift, double m_setpoint) {
    lift = m_lift;
    setpoint = m_setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lift.setLiftGoal(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("LiftSetpoint finished.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lift.atSetpoint();
  }
}
