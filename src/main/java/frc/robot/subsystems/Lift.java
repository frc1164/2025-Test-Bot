// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.NA;
import java.util.function.Function;

import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.numbers.*;

public class Lift extends SubsystemBase {
    private final SparkMax liftMotor;
    private final SparkMaxConfig liftMotorConfig;

    private final LaserCan ToF;
    private final DigitalInput topLim, bottomLim;

    private final ProfiledPIDController liftPID; // either feedforward or switch statement
    private final ElevatorFeedforward liftFeedforward;

    private final LinearSystem<N2, N1, N1> elevatorLinearSystem;
    private final KalmanFilter<N2, N1, N1> elevatorKalmanFilter;
    // private final LinearQuadraticRegulator<N2, N1, N1> elevatorRegulator;
    // private final LinearSystemLoop<N2, N1, N1> elevatorSystemLoop;

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
        liftPID.setTolerance(.1);
        liftPID.setGoal(.225);
        
        liftFeedforward = new ElevatorFeedforward(LiftConstants.liftFeedforwardkS,
        LiftConstants.liftFeedforwardkG, LiftConstants.liftFeedforwardkV,
        LiftConstants.liftFeedforwardkA);
        
        // 20ms or systemtime for dt

        // 1 State Kalman
        // double[] AConst = {1};
        // double[] BConst = {0.02};
        // double[] CConst = {1};
        // double[] DConst = {0};

        // Matrix<N1, N1> A = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), AConst);
        // Matrix<N1, N1> B = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), BConst);
        // Matrix<N1, N1> C = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), CConst);
        // Matrix<N1, N1> D = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), DConst);

        // elevatorLinearSystem = new LinearSystem<N1, N1, N1>(A, B, C, D);

        // Matrix<N1, N1> encoderStateStdev = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), new double[] {0.001});
        // Matrix<N1, N1> encoderMeasureStdev = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), new double[] {0.002});

        // elevatorKalmanFilter = new KalmanFilter<N1, N1, N1>(
        //     Nat.N1(), 
        //     Nat.N1(), 
        //     elevatorLinearSystem, 
        //     encoderStateStdev, 
        //     encoderMeasureStdev, 
        //     0.02
        // );

        // 2 State Kalman
        elevatorLinearSystem = new LinearSystem<N2, N1, N1>(
            LiftConstants.A,
            LiftConstants.B,
            LiftConstants.C,
            LiftConstants.D
        );

        Matrix<N2, N1> encoderStateStdev = new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[] {LiftConstants.statePosStdev, LiftConstants.stateVelStdev});
        Matrix<N1, N1> encoderMeasureStdev = new Matrix<N1, N1>(Nat.N1(), Nat.N1(), new double[] {LiftConstants.measurePosStdev});

        elevatorKalmanFilter = new KalmanFilter<N2, N1, N1>(
            Nat.N2(), 
            Nat.N1(), 
            elevatorLinearSystem, 
            encoderStateStdev, 
            encoderMeasureStdev, 
            0.02
        );

        // elevatorKalmanFilter.setXhat(1, getMotorPosition());
        // elevatorKalmanFilter.setXhat(2, getMotorVelocity());
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
        // return ToF.getMeasurement().distance_mm / 1000.0;
        return elevatorKalmanFilter.getXhat(1);
    }
    
    //input
    public double getMotorVelocity(){
        return liftMotor.get()/9;
    }

    //measurement
    public double getMotorPosition(){
        return liftMotor.getEncoder().getPosition();
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

        //double fusedMeasurement = (getMotorPosition() + ToF.getMeasurement().distance_mm / 1000.0) / 2;

        elevatorKalmanFilter.predict(
            new Matrix<N1, N1>(Nat.N1(), Nat.N1(), new double[] {getMotorVelocity()}),
            0.02
        );

        elevatorKalmanFilter.correct(
            new Matrix<N1, N1>(Nat.N1(), Nat.N1(), new double[] {getMotorVelocity()}),
            new Matrix<N1, N1>(Nat.N1(), Nat.N1(), new double[] {ToF.getMeasurement().distance_mm / 1000.0})
        );

        SmartDashboard.putNumber("Tof Measurement", ToF.getMeasurement().distance_mm / 1000.0);
        // SmartDashboard.putNumber("Kalman Position", elevatorKalmanFilter.getXhat(1));
        // SmartDashboard.putNumber("Kalman Velocity", elevatorKalmanFilter.getXhat(2));
    }
}
