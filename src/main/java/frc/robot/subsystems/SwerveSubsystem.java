// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS.SerialDataType;
//import com.kauailabs.navx.frc.*;

import edu.wpi.first.math.geometry.Pose2d;

//import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
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
    DriveConstants.kFrontLeftAbsoluteEncoderReversed,
    "FrontLeft");

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveCANID, 
    DriveConstants.kFrontRightTurningCANID, 
    DriveConstants.kFrontRightDriveEncoderReversed, 
    DriveConstants.kFrontRightTurningEncoderReversed, 
    DriveConstants.kFrontRightAbsoluteEncoderPort, 
    DriveConstants.kFrontRightAbsoluteEncoderOffsetRad, 
    DriveConstants.kFrontRightAbsoluteEncoderReversed,
    "FrontRight");

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveCANID, 
    DriveConstants.kBackLeftTurningCANID, 
    DriveConstants.kBackLeftDriveEncoderReversed, 
    DriveConstants.kBackLeftTurningEncoderReversed, 
    DriveConstants.kBackLeftAbsoluteEncoderPort, 
    DriveConstants.kBackLeftAbsoluteEncoderOffsetRad, 
    DriveConstants.kBackLeftAbsoluteEncoderReversed,
    "BackLeft");
    
  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveCANID, 
    DriveConstants.kBackRightTurningCANID, 
    DriveConstants.kBackRightDriveEncoderReversed, 
    DriveConstants.kBackRightTurningEncoderReversed, 
    DriveConstants.kBackRightAbsoluteEncoderPort, 
    DriveConstants.kBackRightAbsoluteEncoderOffsetRad, 
    DriveConstants.kBackRightAbsoluteEncoderReversed,
    "BackRight");



  private final WPI_PigeonIMU gyro;
  
  private final SwerveDriveOdometry odometery = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0),
            getModulePositions());
 
 
  public static boolean isFieldOriented = false;


  public SwerveSubsystem(WPI_PigeonIMU gyro ) {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetGyro();
      } catch (Exception e) {
      }
    }).start();

    this.gyro = gyro;
    /* 
    gyro.calibrate();
    if (!gyro.isCalibrating()){
      gyro.zeroYaw();
    }*/

  }

  public void zeroHeading() {
    gyro.reset();

    
    resetOdometry(getPose());
  }

  public void resetGyro(){
    //gyro.calibrate();
    gyro.reset();
  }

  public Pose2d getPose(){
    return odometery.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometery.resetPosition(getRotation2d(), getModulePositions(), pose);
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    gyroUpdate();
  }

  //if the angle is like 400, this will return it as 40, and make it in between 0-360
  public double getHeading() {
    //return Math.IEEEremainder((gyro.getAngle()), 360);
    return gyro.getAngle() * Math.PI / 180;
    //return gyro.getYaw();
  }

  public Rotation2d getRotation2d() {
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

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
    return swerveModulePositions;
  }

  public void gyroUpdate(){
    SmartDashboard.putNumber("1Robot Yaw", (gyro.getYaw()));
    SmartDashboard.putNumber("1Robot Pitch", gyro.getPitch());
    SmartDashboard.putNumber("1Robot Roll", gyro.getRoll());
    SmartDashboard.putNumber("1Robot Angle", gyro.getAngle());
    SmartDashboard.putData(gyro);
  }

  public boolean getIsFieldOrientated(){
    return isFieldOriented;
  }

  public void changeFieldOrientated(){
    if (isFieldOriented){
      isFieldOriented = false;
    } else isFieldOriented = true;
  }

  public Rotation2d getPitch(){
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

  public Rotation2d getRoll(){
    return Rotation2d.fromDegrees(gyro.getRoll());
  }

  public double getTilt(){
    double pitch = getPitch().getDegrees();
    double roll = getRoll().getDegrees();

    if ((pitch + roll >= 0)){
      return Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));
    } else return -Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
    //SmartDashboard.putData("0Rotation2D", getRotation2d());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putBoolean("Is Field Orientated", getIsFieldOrientated());
    gyroUpdate();

    odometery.update(getRotation2d(), getModulePositions());

    SmartDashboard.putNumber("Front Left Module", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Right Module", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Left Module", backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Right Module", backRight.getAbsoluteEncoderRad());

    SmartDashboard.putData(this);


  }
}
