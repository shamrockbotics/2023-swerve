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

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private final CANSparkMax rightElevMotor;
  private  final CANSparkMax leftElevMotor;

  private final RelativeEncoder rightElevEncoder;
  private final RelativeEncoder leftElevEncoder;
  public Elevator() {


    rightElevMotor = new CANSparkMax(Constants.MotorConstants.leftElevatorMotorID, MotorType.kBrushless);
    leftElevMotor = new CANSparkMax(Constants.MotorConstants.rightElevatorMotorID, MotorType.kBrushless);

    rightElevEncoder = rightElevMotor.getEncoder();
    leftElevEncoder = leftElevMotor.getEncoder();


    rightElevMotor.restoreFactoryDefaults();
    leftElevMotor.restoreFactoryDefaults();

    rightElevEncoder.setPositionConversionFactor(Constants.MotorConstants.elevatorPositionEncoderConstant);
    leftElevEncoder.setPositionConversionFactor(Constants.MotorConstants.elevatorPositionEncoderConstant);

    rightElevMotor.setInverted(Constants.MotorConstants.rightElevatorInverted);
    leftElevMotor.setInverted(Constants.MotorConstants.leftElevatorInverted);


    rightElevMotor.setIdleMode(IdleMode.kBrake);
    leftElevMotor.setIdleMode(IdleMode.kBrake);

    resetEncoders();


  }

  public void resetEncoders(){
    rightElevEncoder.setPosition(0);
    leftElevEncoder.setPosition(0);
  }

  public void extend(){
    if (getPosition() > 100){
      stopElev();
    } else {
      rightElevMotor.set(Constants.MotorConstants.elevatorSpeed);
      leftElevMotor.set(Constants.MotorConstants.elevatorSpeed);
    }
    
  }

  

  public void retract(double retract){
    /*if (retract > 0.1){*/
      if (getPosition() < 2){
        if (getPosition() < 0){
          resetEncoders();
        }
        rightElevMotor.set(-Constants.MotorConstants.elevatorSpeed/2);
        leftElevMotor.set(-Constants.MotorConstants.elevatorSpeed/2);
      } else {
        rightElevMotor.set(-Constants.MotorConstants.elevatorSpeed);
        leftElevMotor.set(-Constants.MotorConstants.elevatorSpeed);
      }
    
    
  }

  public void extend(double speed){
    rightElevMotor.set(speed);
    leftElevMotor.set(speed);
  }

  public void stopElev(){
    rightElevMotor.set(0);
    leftElevMotor.set(0);
  }

  public double getPosition(){
    double position = (rightElevEncoder.getPosition() + leftElevEncoder.getPosition())/2;
    return position;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorRight Position", rightElevEncoder.getPosition());
    SmartDashboard.putNumber("ElevatorLeft Position", leftElevEncoder.getPosition());
    SmartDashboard.putNumber("Average Elevator Position", getPosition());
  }
}
