// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDeadband = 0.25;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 18.75;// 7 / 150;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.25;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), /* Left front */
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), /* Right front */
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), /* Left rear */
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /* Right rear */

        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 40;
        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kBackRightDriveMotorPort = 30;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 41;
        public static final int kFrontRightTurningMotorPort = 21;
        public static final int kBackRightTurningMotorPort = 31;

        public static final InvertedValue kFrontLeftTurningEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kBackLeftTurningEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kFrontRightTurningEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kBackRightTurningEncoderReversed = InvertedValue.Clockwise_Positive;

        public static final InvertedValue kFrontLeftDriveEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kBackLeftDriveEncoderReversed = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kFrontRightDriveEncoderReversed = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kBackRightDriveEncoderReversed = InvertedValue.CounterClockwise_Positive;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 42;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackRightDriveAbsoluteEncoderPort = 32;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 51.76764  * Math.PI / 180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 234.9324 * Math.PI / 180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 70.83972 * Math.PI / 180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 37.3536 * Math.PI / 180.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSLeft = 0.30323;
        public static final double kVLeft = 3.5231;
        public static final double kALeft = 0.17608;

        public static final double kSRight = 0.30202;
        public static final double kVRight = 3.5304;
        public static final double kARight = 0.20064;

        // Drive/Rotation gain
        public static final double kRotGain = 2;
        public static final double kDriveGain = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class LimeLightConstants{
        public static final String kLLTags = "limelight-tags";
        public static final String kTagLimelightNetworkTableName = "limelight-tags";
        public static final int kAprilTagPipeline = 0;
    }

    public static final class OffsetConstants{
        public static final double offsetRight = -13.12;
        public static final double offsetLeft = 13.12;



    }


}
