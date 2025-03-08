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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final SparkMax climbMotor;
  private final SparkMaxConfig climbMotorConfig;
  private final RelativeEncoder climbRelativeEncoder;
  private final CANcoder climbAbsoluteEncoder;
  private final CANcoderConfiguration config;
  private final PIDController climbPidController;
  private double bottomLim;

  public Climb() {
    climbMotor = new SparkMax(58, SparkLowLevel.MotorType.kBrushless);
    climbAbsoluteEncoder = new CANcoder(59);
    climbRelativeEncoder = climbMotor.getEncoder();
    config = new CANcoderConfiguration();
    climbMotorConfig = new SparkMaxConfig();

    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    climbMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    climbMotor.configure(climbMotorConfig, null, null);

    climbPidController = new PIDController(0,0,0);

    bottomLim = 85;

  }
  public void setSpeed(double speed){
    if (speed > 0){
      if (climbAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360 < bottomLim){
        speed = speed;
      } else {
        speed = 0;
      }
    }
  }

  public void runPID(){
    setSpeed(climbPidController.calculate(climbAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360,  bottomLim));
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
