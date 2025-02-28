// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final SparkMax armMotor;
  private final SparkMaxConfig motorConfig;

  private final CANcoder absEncoder;
  private final CANcoderConfiguration encoderConfig;

  private final DigitalInput beamBrake;

  private final PIDController armPID;

  public Arm() {
    armMotor = new SparkMax(57, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.inverted(false)
               .idleMode(IdleMode.kBrake);
    armMotor.configure(motorConfig, null, null);

    absEncoder = new CANcoder(58);
    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absEncoder.getConfigurator().apply(encoderConfig);
    //create an absolute encoder offset, 0 point should be straight down

    beamBrake = new DigitalInput(3);

    armPID = new PIDController(0, 0, 0);
  }

  public double getAbsoluteEncoderRad(){
    return absEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
  }

  public void runArm(Double speed){
    if (getAbsoluteEncoderRad() > 0 && getAbsoluteEncoderRad() < 5/4 * Math.PI){
      armMotor.set(speed);
    } else {
      armMotor.set(0);
    }
  }

  public void runPID() {
    runArm(-armPID.calculate(getAbsoluteEncoderRad()));
  }

  // public void setPID(ArmConstants.Setpoint setpoint){
  //  switch (setpoint) {
  //   case PICKUP: armPID.setSetpoint(0);  armPID.setP(0.002);  armPID.setI(0.00003);
  //   break;
  //   case L2: armPID.setSetpoint(3/4 * Math.PI);   armPID.setP(.05);  armPID.setI(.0005);
  //   break;
  //   //add case L3 and L4
  // }
  // }

  @Override
  public void periodic() {
    runPID();
  }
}
