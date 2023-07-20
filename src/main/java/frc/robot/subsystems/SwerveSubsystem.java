// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveCANID, 
    DriveConstants.kFrontLeftTurningCANID, 
    DriveConstants.kFrontLeftDriveEncoderReversed, 
    DriveConstants.kFrontLeftTurningEncoderReversed, 
    DriveConstants.kFrontLeftAbsoluteEncoderPort, 
    DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad, 
    DriveConstants.kFrontLeftAbsoluteEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveCANID, 
    DriveConstants.kFrontRightTurningCANID, 
    DriveConstants.kFrontRightDriveEncoderReversed, 
    DriveConstants.kFrontRightTurningEncoderReversed, 
    DriveConstants.kFrontRightAbsoluteEncoderPort, 
    DriveConstants.kFrontRightAbsoluteEncoderOffsetRad, 
    DriveConstants.kFrontRightAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveCANID, 
    DriveConstants.kBackLeftTurningCANID, 
    DriveConstants.kBackLeftDriveEncoderReversed, 
    DriveConstants.kBackLeftTurningEncoderReversed, 
    DriveConstants.kBackLeftAbsoluteEncoderPort, 
    DriveConstants.kBackLeftAbsoluteEncoderOffsetRad, 
    DriveConstants.kBackLeftAbsoluteEncoderReversed);
    
  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveCANID, 
    DriveConstants.kBackRightTurningCANID, 
    DriveConstants.kBackRightDriveEncoderReversed, 
    DriveConstants.kBackRightTurningEncoderReversed, 
    DriveConstants.kBackRightAbsoluteEncoderPort, 
    DriveConstants.kBackRightAbsoluteEncoderOffsetRad, 
    DriveConstants.kBackRightAbsoluteEncoderReversed);


  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);
  //private final AHRS gyro = new AHRS(SPI.Port.kMSP);


  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();



  }

  public void zeroHeading() {
    gyro.reset();
  }

  //if the angle is like 400, this will return it as 40, and make it in between 0-360
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d geRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());

    SmartDashboard.putNumber("Front Left Module", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Right Module", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Left Module", backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Right Module", backRight.getAbsoluteEncoderRad());


  }
}
