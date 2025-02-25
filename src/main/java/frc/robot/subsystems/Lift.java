// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
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

public class Lift extends SubsystemBase {
  private final SparkMax liftMotor;
  private final SparkMaxConfig liftMotorConfig;
  private final RelativeEncoder liftRelativeEncoder;

  private final CANcoder liftAbsoluteEncoder;
  private final CANcoderConfiguration config;

  private final LaserCan tofL, tofR;
  private final DigitalInput topLim, bottomLim;

  private final PIDController liftPidController; //migth become a trapezoid profiled one not sure yet
  private double kp, ki, kd;
  /** Creates a new Lift. */
  public Lift() {
    liftMotor = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);

    liftMotorConfig = new SparkMaxConfig();
    liftMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    liftMotor.configure(liftMotorConfig, null, null);

    liftRelativeEncoder = liftMotor.getEncoder();

    liftAbsoluteEncoder = new CANcoder(54);
    config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    liftAbsoluteEncoder.getConfigurator().apply(config);

    tofL = new LaserCan(55);
    tofR = new LaserCan(56);
    topLim = new DigitalInput(0);
    bottomLim = new DigitalInput(1);


    liftPidController = new PIDController(kp, ki, kd);
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
  
    


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
