// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d yawPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public double yawRate = 0;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void resetHeading() {}

  public default void setAngleAdjustment(double angle) {}

  public default void updateInputs(GyroIOInputs inputs) {}
}