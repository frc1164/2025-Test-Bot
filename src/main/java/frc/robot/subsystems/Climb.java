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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final SparkMax climbMotor;
  private final SparkMaxConfig climbMotorConfig;
  private final RelativeEncoder climbRelativeEncoder;
  private final PIDController climbPidController;
  private double bottomLim;
  private final DigitalInput verticalLimitSwitch;
  public boolean runGate;

  public Climb() {
    climbMotor = new SparkMax(58, SparkLowLevel.MotorType.kBrushless);
    climbRelativeEncoder = climbMotor.getEncoder();

    climbMotorConfig = new SparkMaxConfig();
    climbMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    climbMotor.configure(climbMotorConfig, null, null);

    climbPidController = new PIDController(0,0,0);
    verticalLimitSwitch = new DigitalInput(3);
    
    runGate = false;
    // // bottomLim = 85;
    bottomLim = 40;
  }
  public void setSpeed(double speed){
    if (speed != 0){
      if (climbRelativeEncoder.getPosition() * 360 / 125 < bottomLim){
        speed = speed;
      } else {
        speed = 0;
      }
    }
  }

  // public void runPID(){
  //   setSpeed(climbPidController.calculate(climbRelativeEncoder.getPosition() * 360,  bottomLim));
  // }

  public boolean getLimitSwitch() {
    // Not sure which is it. Try the one that is commented out if the encoder resets everywhere but vertical.
    // return verticalLimitSwitch.get()    
    return verticalLimitSwitch.get();
  }

  
  @Override
  public void periodic() {
    // If the limit switch is pressed, reset the relative encoder to 0.    
    if(getLimitSwitch()) {
      climbRelativeEncoder.setPosition(0);
    }

    SmartDashboard.putBoolean("climbGate", runGate);
    SmartDashboard.putBoolean("climbLim", getLimitSwitch());
  }
 }
