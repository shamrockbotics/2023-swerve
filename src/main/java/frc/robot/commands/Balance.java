// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class Balance extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  //private WPI_PigeonIMU gyro;
  private final AHRS gyro;
  private final SwerveDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;

  private PIDController balancePID = Constants.AutoConstants.BalancePID;
  
  /** Creates a new Balance. */
  public Balance(SwerveSubsystem swerveSubsystem, AHRS gyro /*WPI_PigeonIMU gyro */) {
    this.swerveSubsystem = swerveSubsystem;
    this.gyro = gyro;
    addRequirements(swerveSubsystem);
  }

  private void drive(ChassisSpeeds speeds){
    var states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    swerveSubsystem.setModuleStates(states);
  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    double roll = swerveSubsystem.getRoll().getDegrees();
    double pitch = swerveSubsystem.getPitch().getDegrees();

    double vx = balancePID.calculate(roll, -2.6);
    double vy = balancePID.calculate(pitch, -2.6);

    drive(new ChassisSpeeds(vx, vy, 0));
  }


  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
