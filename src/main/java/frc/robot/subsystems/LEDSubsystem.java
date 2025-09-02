// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDSubsystem extends SubsystemBase {
  Distance LED_SPACING = Meters.of(1.0 / 60);
  private static final int kPort = 3;
  // private static final int kLength = 121;
  private static final int kLength = 73;


  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final AddressableLEDBufferView m_left;
  private final AddressableLEDBufferView m_right;
  // private final AddressableLEDBufferView m_ziaCenter;
  // private final AddressableLEDBufferView m_ziaArms;

  private LEDPattern pattern1;
  private LEDPattern pattern2;
  private LEDPattern pattern3;
  private LEDPattern pattern4;
  Color purple, orange;
  private final Lift m_Lift;
  private final Arm m_Arm;

  public LEDSubsystem(Lift Lift, Arm arm) {
    purple = new Color(135, 0, 211);
    orange = new Color(255, 20, 0);
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    // m_ziaCenter = m_buffer.createView(0,7);
    // m_ziaArms = m_buffer.createView(8,47);
    // m_left = m_buffer.createView(48,87);
    // m_right = m_buffer.createView(88,120);
    m_left = m_buffer.createView(0,39);
    m_right = m_buffer.createView(40,72);
    m_Lift = Lift;
    m_Arm = arm;

    pattern1 = height(colorOrange());
    pattern2 = height(colorPurple());
    pattern3 = colorPurple();
    pattern4 = colorOrange();
    m_led.start();

    

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!


  }

  public LEDPattern colorOrange(){
    return LEDPattern.solid(orange);
  }

  public LEDPattern colorPurple(){
    return LEDPattern.solid(purple);
  }

  public LEDPattern colorWhite(){
    return LEDPattern.solid(Color.kBlanchedAlmond);
  }

  
  public LEDPattern colorGreen(){
    return LEDPattern.solid(Color.kGreen);
  }

  public LEDPattern scrollingRainbow(){
    LEDPattern m_rainbow = LEDPattern.rainbow(255,100);
    LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.08), LED_SPACING);
    return m_scrollingRainbow;
  }
  //turns off scrollingRainbow when 

  public LEDPattern height(LEDPattern color){
    LEDPattern pattern; 
    LEDPattern base = color;
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> m_Lift.getLiftHeight() * (10.0 / 7.0));
    mask.blend(colorOrange());
    pattern = base.mask(mask);
    return pattern;
  }

  public void setPattern1(LEDPattern pattern){
    pattern1 = pattern;
  }

  public LEDPattern getPattern1(){
    return pattern1;
  }

  public void setPattern2(LEDPattern pattern){
    pattern2 = pattern;
  }

  public LEDPattern getPattern2(){
    return pattern2;
  }

  public void setPattern3(LEDPattern pattern){
    pattern3 = pattern;
  }

  public LEDPattern getPattern3(){
    return pattern3;
  }

  public void setPattern4(LEDPattern pattern){
    pattern4 = pattern;
  }

  public LEDPattern getPattern4(){
    return pattern4;
  }

  public void haveCoral(){
    if (m_Arm.getIntake() && getPattern3() == scrollingRainbow()){
      setPattern3(colorPurple());
      setPattern4(colorOrange());
    }
    else if (m_Arm.getIntake()){
      setPattern3(colorGreen());
      setPattern4(colorGreen());
    }
    else{
      setPattern3(colorPurple());
      setPattern4(colorOrange());
    }
    }
    //test this to make sure zia symbol still defaults to purple and orange 
  

  public void applyPattern(LEDPattern pattern1, LEDPattern pattern2, LEDPattern pattern3, LEDPattern pattern4){
    // pattern3.applyTo(m_ziaCenter);
    // pattern4.applyTo(m_ziaArms);
    pattern1.applyTo(m_left);
    pattern2.applyTo(m_right);
    m_led.setData(m_buffer);
  }


  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    haveCoral(); //test overlap w/ pathfind light
    applyPattern(getPattern1(), getPattern2(), getPattern3(), getPattern4());
    SmartDashboard.putString("pattern1", pattern1.toString());
    m_led.setData(m_buffer);
    
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }

}
