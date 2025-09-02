// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmScorePosition;

public class Arm extends SubsystemBase {
  private final SparkMax armMotor;
  private final SparkMaxConfig motorConfig;
  private final AbsoluteEncoder absoluteEncoder;
  private final AbsoluteEncoderConfig config;
  //private final AbsoluteEncoderConfigAccessor accessor;


  private final DigitalInput beamBrake, isHeld;

  private final PIDController armPID;
  private final ArmFeedforward armFeedforward;

  public Arm() {
    armMotor = new SparkMax(57, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.inverted(true)
               .idleMode(IdleMode.kBrake);

    absoluteEncoder = armMotor.getAbsoluteEncoder();
    config = new AbsoluteEncoderConfig();
    config.positionConversionFactor(Math.PI * 2)  
          .velocityConversionFactor(Math.PI / 30)
          .zeroOffset(0.312)
          .inverted(true);
    motorConfig.apply(config);

    armMotor.configure(motorConfig, null, null);

    beamBrake = new DigitalInput(5);
    isHeld = new DigitalInput(0);

    // armPID = new ProfiledPIDController(
    //     ArmConstants.kP,
    //     ArmConstants.kI,
    //     ArmConstants.kD, 
    //     new Constraints(
    //         ArmConstants.maxVelocity,
    //         ArmConstants.maxAcceleration));

    // armPID.setGoal(Math.PI/2.0);
    
    armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    armPID.setTolerance((Math.PI/180) * 18);
    armPID.setSetpoint(ArmConstants.Up);
    armFeedforward = new ArmFeedforward(ArmConstants.kS,
        ArmConstants.kG, ArmConstants.kV,
        ArmConstants.kA);
  }

  public void runArm(double voltage){
    if (voltage > 0){
      if (absoluteEncoder.getPosition() <= 5){
        armMotor.set(voltage);
      } else {
        armMotor.set(0);
      }
    }
    if (voltage < 0){
      if (absoluteEncoder.getPosition() >= 1){
        armMotor.set(voltage);
      }else{
        armMotor.set(0);
      }
    }
  }

  // public double getFeedforwardPIDOutput() {
  //   double feedforwardOutput = armFeedforward.calculate(absoluteEncoder.getPosition() + ArmConstants.charOffset, armPID.getSetpoint().velocity);
  //   double armPIDOutput = armPID.calculate(absoluteEncoder.getPosition());
  //   return (armPIDOutput);
  // }

  public void setGoal(double setpoint){
    //armPID.setGoal(setpoint);
    armPID.setSetpoint(setpoint);
  }
  
  public boolean getIntake(){
    return beamBrake.get();
  }


  public boolean atSetpoint(){
    return (Math.abs(absoluteEncoder.getPosition() - armPID.getSetpoint()) < .5);
  }

  public double getPosition(){
    return absoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    runArm(armPID.calculate(absoluteEncoder.getPosition()));
    SmartDashboard.putNumber("armEncoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("armCmd", armPID.calculate(absoluteEncoder.getPosition()));
    SmartDashboard.putBoolean("beam", getIntake());
    SmartDashboard.putNumber("Difference", absoluteEncoder.getPosition() - armPID.getSetpoint());
  }
}
