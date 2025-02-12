// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDS extends Command {
  private final CommandXboxController m_CommandXboxController;
  private final LEDSubsystem m_LedSubsystem;
  
  /** Creates a new LEDS. */
  
  public LEDS(LEDSubsystem ledSubsystem, CommandXboxController controller) {
    m_LedSubsystem = ledSubsystem; 
    m_CommandXboxController = controller;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_LedSubsystem);
  }

 
 
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LEDPattern pattern = LEDPattern.progressMaskLayer(() -> m_CommandXboxController.getRightTriggerAxis());
    SmartDashboard.putNumber("rt", m_CommandXboxController.getRightTriggerAxis());

    m_LedSubsystem.runPattern(pattern);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
