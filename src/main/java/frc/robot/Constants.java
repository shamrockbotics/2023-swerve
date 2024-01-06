// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final int OperatorControllerPort = 1;
    public static final double kDeadband = 0.1;

    public static final int intakeButton = 2;
    public static final int extakeButton = 1;
    public static final int extendButton = 6;
    public static final int retractButton = 5;
    public static final int wristButton = 12;


    public static final int playerStationButton = 10;
    public static final int highScoreButton = 7;
    public static final int middleScoreButton = 9;
    public static final int groundScoreButton = 11;
  }

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 6;
    public static final double kPYController = 6;
    public static final double kPThetaController = 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);


    public static final double ElevHigh = 100;

    public static PIDController BalancePID = new PIDController(0.05, 0, 0);
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
    public static final double kPTurning = 0.255;
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


    public static final double kPhysicalMaxSpeedMetersPerSecond = 1; // 5

    public static final double kTeleDriveMaxAccelerationUnitesPerSecond = 1; // 3
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 1; // 3
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5; // 3
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 1.25;

    public static final int kFrontLeftDriveCANID = 4;
    public static final int kFrontLeftTurningCANID = 3;
    public static final int kFrontRightDriveCANID = 5;
    public static final int kFrontRightTurningCANID = 6;
    public static final int kBackLeftDriveCANID = 2;//2
    public static final int kBackLeftTurningCANID = 1;
    public static final int kBackRightDriveCANID = 8;
    public static final int kBackRightTurningCANID = 7;

    public static final boolean kFrontLeftDriveEncoderReversed = true; 
    public static final boolean kFrontLeftTurningEncoderReversed = false; 
    public static final boolean kFrontRightDriveEncoderReversed = true; 
    public static final boolean kFrontRightTurningEncoderReversed = false; 
    public static final boolean kBackLeftDriveEncoderReversed = false; 
    public static final boolean kBackLeftTurningEncoderReversed = false; 
    public static final boolean kBackRightDriveEncoderReversed = false; 
    public static final boolean kBackRightTurningEncoderReversed = false; 

    public static final int kFrontLeftAbsoluteEncoderPort = 1;
    public static final int kFrontRightAbsoluteEncoderPort = 0;
    public static final int kBackLeftAbsoluteEncoderPort = 2;
    public static final int kBackRightAbsoluteEncoderPort = 3;

    //offset values
    public static final double kFrontLeftAbsoluteEncoderOffsetRad = 1.733;//1.720;//1.689;
    public static final double kFrontRightAbsoluteEncoderOffsetRad = -1.489;//-1.503;//-1.495;
    public static final double kBackLeftAbsoluteEncoderOffsetRad = 1.655;//-1.487;//-1.475;
    public static final double kBackRightAbsoluteEncoderOffsetRad = 2.735;//-0.393;//-2.423; 0.39
    public static final boolean kFrontLeftAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftAbsoluteEncoderReversed = true;
    public static final boolean kBackRightAbsoluteEncoderReversed = true;

  }

  public static class MotorConstants{
    public static final int intakeid = 9;
    public static final double intakeSpeed = -0.75;
    public static final boolean intakeInverted = false;

    public static final int leftElevatorMotorID = 10;
    public static final int rightElevatorMotorID = 11;
    public static final double elevatorSpeed = 0.2;
    public static final boolean rightElevatorInverted = true;
    public static final boolean leftElevatorInverted = false;
    public static final double elevatorPositionEncoderConstant = 2 * Math.PI / 16;
    //circumfrence of the motor shaft divided by the gearbox ratio
    public static final double scoreHighElev = 0;
    public static final double scoreMidElev = 0;
    public static final double scoreGroundElev = 0;
    public static final double humanElev = 0;

    public static final int leftWristMotorID = 12;
    public static final int rightWristMotorID = 13;
    public static final double wristMotorSpeed = 0.1;
    public static final boolean leftWristInverted = false;
    public static final boolean rightwristInverted = true;
    public static final double wristPositionEncoderConstant = 360 * (9/256);
    public static final double scoreHighWrist = 0;
    public static final double scoreMidWrist = 0;
    public static final double scoreGroundWrist = 0;
    public static final double humanWrist = 0;
  }


  public static class LEDConstants{
    public static final int[][][] flags = {

      { // rainbow flag
        {0, 0, 0}, // red
        {0, 0, 0}, // orange
        {0, 0, 0}, // yellow
        {0, 0, 0}, // green
        {0, 0, 0}, // blue
        {0, 0, 0} // purple
      }, // end of rainbow flag

      { // bi flag
        {0, 0, 0}, // pink
        {0, 0, 0}, // purple
        {0, 0, 0} // blue
      } // end of bi flag


    }; // end of flags array
  }
}
