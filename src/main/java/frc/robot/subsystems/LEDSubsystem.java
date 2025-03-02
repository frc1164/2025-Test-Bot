// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 74;


  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final AddressableLEDBufferView m_left;
  private final AddressableLEDBufferView m_right;
  // private final AddressableLEDBufferView m_ziaCenter;
  // private final AddressableLEDBufferView m_ziaArms;

  

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_left = m_buffer.createView(0,7);
    m_right = m_buffer.createView(8,73);
    // m_ziaCenter = m_buffer.createView(0,7);
    // m_ziaArms = m_buffer.createView(8,47);
    m_led.start();

    

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    //setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlue)).withName("Off"));


  }

  // public double getToF(){
  //   Measurement measurement = ToF.getMeasurement();
  //   return measurement.distance_mm;
  // }



  public void applyPattern(LEDPattern pattern1, LEDPattern pattern2){
    // pattern1.applyTo(m_ziaCenter);
    // pattern2.applyTo(m_ziaArms);
    pattern1.applyTo(m_left);
    pattern2.applyTo(m_right);
    m_led.setData(m_buffer);
  }


  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
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

public int getToF() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getToF'");
}
}
