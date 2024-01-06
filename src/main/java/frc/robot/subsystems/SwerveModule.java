// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;

  private final AnalogEncoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private final String name;

  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, 
    boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {

    this.name = name;
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogEncoder(absoluteEncoderID);
        
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();
    

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2Meter);
    turningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2MeterPerSec);

    turningPidController = new PIDController(Constants.ModuleConstants.kPTurning, Constants.ModuleConstants.kITurning, Constants.ModuleConstants.kDTurning);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);

    resetEncoders();
    //SmartDashboard.putData(name + " PID Controller", turningPidController);
  }



  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getAbsolutePosition();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    if (angle < -Math.PI){
      angle += 2*Math.PI;
    } else if (angle > Math.PI){
      angle -= 2*Math.PI;
    }
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }


  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
  } 

  public SwerveModulePosition getPosition(){
    //SwerveModulePosition modulePosition;
    //double position = (driveEncoder.getPosition()*Math.PI*Constants.ModuleConstants.kWheelDiameterMeters)*0.0254;
    return new SwerveModulePosition(driveEncoder.getPosition(), getState().angle);
    //return modulePosition;
  }

  public void setDesiredState(SwerveModuleState state){
    /*if (Math.abs(state.speedMetersPerSecond) < 0.00001){
      stop();
      return;
    }*/

    state = SwerveModuleState.optimize(state, getState().angle);
    SmartDashboard.putNumber(name + " Current Angle", getState().angle.getRadians());
    SmartDashboard.putNumber(name + " Desired Angle", state.angle.getRadians());
    double turningMotorCalculate = turningPidController.calculate(getState().angle.getRadians(), state.angle.getRadians());
    SmartDashboard.putNumber(name + " Turning Motor Output", turningMotorCalculate);
    turningMotor.setIdleMode(IdleMode.kCoast);
    driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningMotorCalculate);
    
    //SmartDashboard.putNumber(name + " State Speed", Math.abs(state.speedMetersPerSecond));
    //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop(){
    
    driveMotor.set(0);
    turningMotor.set(0);
  }

  

}
