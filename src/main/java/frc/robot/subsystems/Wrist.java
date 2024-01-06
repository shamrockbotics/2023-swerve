// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  private final CANSparkMax leftWristMotor;
  private final CANSparkMax rightWristMotor;

  private final RelativeEncoder leftWristEncoder;
  private final RelativeEncoder rightWristEncoder;

  public Wrist() {

    leftWristMotor = new CANSparkMax(Constants.MotorConstants.leftWristMotorID, MotorType.kBrushless);
    rightWristMotor = new CANSparkMax(Constants.MotorConstants.rightWristMotorID, MotorType.kBrushless);

    leftWristEncoder = leftWristMotor.getEncoder();
    rightWristEncoder = rightWristMotor.getEncoder();

    leftWristMotor.restoreFactoryDefaults();
    rightWristMotor.restoreFactoryDefaults();

    leftWristMotor.setInverted(Constants.MotorConstants.leftWristInverted);
    rightWristMotor.setInverted(Constants.MotorConstants.rightwristInverted);

    leftWristEncoder.setPositionConversionFactor(Constants.MotorConstants.wristPositionEncoderConstant);
    rightWristEncoder.setPositionConversionFactor(Constants.MotorConstants.wristPositionEncoderConstant);

    leftWristMotor.setIdleMode(IdleMode.kBrake);
    rightWristMotor.setIdleMode(IdleMode.kBrake);

    resetEncoders();
  }

  public void resetEncoders(){
    leftWristEncoder.setPosition(0);
    rightWristEncoder.setPosition(0);
  }

  public void rotateWrist(double speed){
    leftWristMotor.set(speed);
    rightWristMotor.set(speed);
  }
  
  public void stopWrist(){
    leftWristMotor.set(0);
    rightWristMotor.set(0);
  }
  
  public double getAngle(){
    double position = (rightWristEncoder.getPosition() + -leftWristEncoder.getPosition())/2;
    return position*100;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", getAngle());
  }
}
