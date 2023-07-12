// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
    

    //SwerveDrive Konstants
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final double kp = 0.25;
    public static final double ki = 0;
    public static final double kd = 0;

    public static final int left1DriveMotor = 4;
    public static final int left1TurnMotor = 3;

    public static final int left2DriveMotor = 2;
    public static final int left2TurnMotor = 1;

    public static final int right1DriveMotor = 5;
    public static final int right1TurnMotor = 6;

    public static final int right2DriveMotor = 8;
    public static final int right2TurnMotor = 7;

    public static final int left1AbsoluteEncoder = 1;
    public static final int left2AbsoluteEncoder = 2;
    public static final int right1AbsoluteEncoder = 0;
    public static final int right2AbsoluteEncoder = 3;



    //Controller Ports
    public static final int DriveControllerPort = 0;
    public static final int OperatorControllerPort = 1;
  
}
