// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Lift extends SubsystemBase {
    private final SparkMax liftMotor;
    private final SparkMaxConfig liftMotorConfig;

    private final LaserCan ToF;
    private final DigitalInput topLim, bottomLim;

    private final ProfiledPIDController liftPID; // either feedforward or switch statement
    private final ElevatorFeedforward liftFeedforward;

    /** Creates a new Lift. */
    public Lift() {
        liftMotor = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);

        liftMotorConfig = new SparkMaxConfig();
        liftMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        liftMotor.configure(liftMotorConfig, null, null);

        // abs encoder offset should make 0 the bottom of the range

        ToF = new LaserCan(55);
        topLim = new DigitalInput(1);
        bottomLim = new DigitalInput(2);

        liftPID = new ProfiledPIDController(
                LiftConstants.liftPIDkP,
                LiftConstants.liftPIDkI,
                LiftConstants.liftPIDkD,
                new Constraints(
                        LiftConstants.liftMaxVelocity,
                        LiftConstants.liftMaxAcceleration));
        liftPID.setGoal(.225);
        
        liftFeedforward = new ElevatorFeedforward(LiftConstants.liftFeedforwardkS,
        LiftConstants.liftFeedforwardkG, LiftConstants.liftFeedforwardkV,
        LiftConstants.liftFeedforwardkA);

    }

    public void runLift(double motorOutput) {
        if (motorOutput > 0) {
            if (!topLim.get()) {
                SmartDashboard.putNumber("motorOutput", motorOutput);
                liftMotor.setVoltage(motorOutput);
            } else {
                liftMotor.setVoltage(0);
            }
        }
        if (motorOutput < 0) {
            if (!bottomLim.get()) {
                SmartDashboard.putNumber("motorOutput", motorOutput);
                liftMotor.setVoltage(motorOutput);
            } else {
                liftMotor.setVoltage(0);
            }
        }
    }

    public boolean atSetpoint(){
        return liftPID.atGoal();
    }
    /*
     * The output of the Feedforward and PID combined (added).
     */
    private double getFeedforwardPIDOutput() {
        double feedforwardOutput = liftFeedforward.calculate(liftPID.getSetpoint().velocity);
        double liftPIDOutput = liftPID.calculate(getLiftHeight());
        return (feedforwardOutput + liftPIDOutput);
    }

    /*
     * Sets the end goal of the profiled PID.
     * This is similar to a normal PID controller setpoint.
     */
    public void setLiftGoal(double goal) {
        liftPID.setGoal(goal);
    }

    public double getGoal(){
        return liftPID.getGoal().position;
    }

    /*
     * Returns the calculated height of the lift in M as a double.
     */
    public double getLiftHeight() {
        // Probably read the (unimplemented) Kalman filter here
        return ToF.getMeasurement().distance_mm / 1000.0;
    }
    

    @Override
    public void periodic() {
        runLift(getFeedforwardPIDOutput());
        SmartDashboard.putNumber("PID Goal", liftPID.getGoal().position);
        SmartDashboard.putNumber("PID Setpoint", liftPID.getSetpoint().position);
        SmartDashboard.putData("liftPID", liftPID);
        SmartDashboard.putNumber("lift Output", getFeedforwardPIDOutput());
        SmartDashboard.putNumber("height", getLiftHeight());
        SmartDashboard.putBoolean("toplim", !topLim.get());
        SmartDashboard.putBoolean("bottomlim", !bottomLim.get());
        
    }
}
