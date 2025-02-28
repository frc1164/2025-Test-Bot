// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
  private final SparkMax liftMotor;
  private final SparkMaxConfig liftMotorConfig;

  private final CANcoder liftAbsoluteEncoder;
  private final CANcoderConfiguration config;

  private final LaserCan tofL, tofR;
  private final DigitalInput topLim, bottomLim;

  private final PIDController liftPID; //either feedforward or switch statement
  private double kp, ki, kd;
  /** Creates a new Lift. */
  public Lift() {
    liftMotor = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);

    liftMotorConfig = new SparkMaxConfig();
    liftMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    liftMotor.configure(liftMotorConfig, null, null);


    liftAbsoluteEncoder = new CANcoder(54);
    config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    liftAbsoluteEncoder.getConfigurator().apply(config);
    //abs encoder offset should make 0 the bottom of the range

    tofL = new LaserCan(55);
    tofR = new LaserCan(56);
    topLim = new DigitalInput(0);
    bottomLim = new DigitalInput(1);


    liftPID = new PIDController(kp, ki, kd);
  }

    public void setLift(double speed) {
      if (speed < 0) {
        if (!topLim.get()) {
            liftMotor.set(0);   
        } else {
            liftMotor.set(speed);
        }
      }
      if (speed > 0){
        if (!bottomLim.get()) {
            liftMotor.set(0);     
        } else {
            liftMotor.set(speed);       
        }
      }
    }
  /*
    public void runLiftPID() {
    setLift(-LiftSetpoint.calculate(currentFilteredHeight()));
  }


  //TUNE THESE PID GAINS THE UP WILL SHATTER AND THE BOTTOM WILL OVERRUN BE CAREFUL
  public void setLiftPID(LiftConstants.Setpoint m_Setpoint) {

    LiftConstants.Setpoint setpoint = m_Setpoint;
    switch (setpoint) {
        case L4: kp = 0.00287; ki = 0.000875; kd = 0.00007; height = LiftConstants.AmpHeight; speed = .225 ;
      break;
        case L3: kp = 0.00475; ki = 0.00115; kd = 0.000085; height = LiftConstants.SpeakerHeight; speed = 1 ;
      break;
        case L2: kp = 0; ki = 0; kd = 0; height = LiftConstants.Stow; speed = 0;
      break;
        case STOW : kp = 0; ki = 0; kd = 0; height = LiftConstants.ClimbTop; speed = 0 ;
      break;
        case PICKUP: kp = .002; ki = 0.0003; kd = 0; height = LiftConstants.PickupHeight; speed = 0  ;

        default:kp = 0; ki = 0; kd = 0; height = LiftConstants.Stow; speed = 0 ;
    } 
    liftPID = new PIDController(kp, ki, kd);
    liftPID.setSetpoint(height);  
  }

  public boolean atSetpoint() {
    int tolerance = 5;
    // if(currentHeight() < getCommandedHeight() + tolerance && currentHeight() > getCommandedHeight() - tolerance) {
    //   return true;
    // }
    if(currentFilteredHeight() < getCommandedHeight() + tolerance && currentFilteredHeight() > getCommandedHeight() - tolerance) {
      return true;
    }
    return false;
  }
    */

  @Override
  public void periodic() {
    //runLiftPID();
  }
}
