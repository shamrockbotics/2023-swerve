// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
  }

  public static class ModuleConstants {

    //how many inches is the wheel diameter
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    //find gear ratio
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 6.75;

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;

    public static final double kTurningEncoderRot2Meter = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPM2MeterPerSec = kTurningEncoderRot2Meter/60;

    //P in PID Controller for Turning Motor
    public static final double kPTurning = 0.25;
    public static final double kITurning = 0;
    public static final double kDTurning = 0;

    public static final double PIDDeadband = 0.1;
  }

  public static class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(20);
    //Distance between left and right wheels
    public static final double kWheelBase = Units.inchesToMeters(20);
    //Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      //Front Left
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      //Front Right
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      //Back Left
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //Back Right
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    );


    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final double kTeleDriveMaxAccelerationUnitesPerSecond = 3;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;

    public static final int kFrontLeftDriveCANID = 4;
    public static final int kFrontLeftTurningCANID = 3;
    public static final int kFrontRightDriveCANID = 5;
    public static final int kFrontRightTurningCANID = 6;
    public static final int kBackLeftDriveCANID = 2;
    public static final int kBackLeftTurningCANID = 1;
    public static final int kBackRightDriveCANID = 8;
    public static final int kBackRightTurningCANID = 7;

    public static final boolean kFrontLeftDriveEncoderReversed = true; 
    public static final boolean kFrontLeftTurningEncoderReversed = false; 
    public static final boolean kFrontRightDriveEncoderReversed = true; 
    public static final boolean kFrontRightTurningEncoderReversed = false; 
    public static final boolean kBackLeftDriveEncoderReversed = true; 
    public static final boolean kBackLeftTurningEncoderReversed = false; 
    public static final boolean kBackRightDriveEncoderReversed = true; 
    public static final boolean kBackRightTurningEncoderReversed = false; 

    public static final int kFrontLeftAbsoluteEncoderPort = 1;
    public static final int kFrontRightAbsoluteEncoderPort = 0;
    public static final int kBackLeftAbsoluteEncoderPort = 2;
    public static final int kBackRightAbsoluteEncoderPort = 3;

    //offset values
    public static final double kFrontLeftAbsoluteEncoderOffsetRad = 1.675;
    public static final double kFrontRightAbsoluteEncoderOffsetRad = -1.65;
    public static final double kBackLeftAbsoluteEncoderOffsetRad = -1.463;
    public static final double kBackRightAbsoluteEncoderOffsetRad = -2.023;

    public static final boolean kFrontLeftAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftAbsoluteEncoderReversed = true;
    public static final boolean kBackRightAbsoluteEncoderReversed = true;

  }
}
