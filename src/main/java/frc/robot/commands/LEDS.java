// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.util.Color;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDS extends Command {
  private final LEDSubsystem m_LedSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;
  private LEDPattern m_rainbow;
  private LEDPattern m_scrollingRainbow;
    Distance LED_SPACING = Meters.of(1.0 / 60);
  
    /** Creates a new LEDS. */
    
    public LEDS(LEDSubsystem ledSubsystem, SwerveSubsystem swerveSubsystem) {
      m_LedSubsystem = ledSubsystem;
      m_SwerveSubsystem = swerveSubsystem; 
      
  
      // Use addRequirements() here to declare subsystem dependencies.
  
      addRequirements(m_LedSubsystem);
    }
  
   
   
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      //SOLID COLOR + SCROLLING RAINBOW
      //LEDPattern base = LEDPattern.solid(Color.kPurple);
      // LEDPattern mask = LEDPattern.progressMaskLayer(() -> m_SwerveSubsystem.getToF()/1000);
      // LEDPattern pattern1 = base.mask(mask);
      // LEDPattern m_rainbow = LEDPattern.rainbow(255,100);
      // m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.08), LED_SPACING);
      // m_LedSubsystem.applyPattern(pattern1, m_scrollingRainbow);

      //ZIA SYMBOL
      // Color purple = new Color(135, 0, 211);
      // Color orange = new Color(255, 20, 0);
      // LEDPattern purpleArms = LEDPattern.solid(purple);
      // LEDPattern orangeCenter = LEDPattern.solid(orange);
      // m_LedSubsystem.applyPattern(orangeCenter, purpleArms);

      // TOF SENSOR WOBBLE
      Color purple = new Color(135, 0, 211);
      LEDPattern leftOn = LEDPattern.solid(purple);
      LEDPattern rightOn = LEDPattern.solid(purple);
      LEDPattern leftOff = LEDPattern.solid(Color.kBlack);
      LEDPattern rightOff = LEDPattern.solid(Color.kBlack);
    
      Boolean lefttrue = m_SwerveSubsystem.getToF() < 500;
      if(lefttrue){
        m_LedSubsystem.applyPattern(leftOn, rightOff);

      } else {
        m_LedSubsystem.applyPattern(leftOff, rightOn);
      }
    

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
