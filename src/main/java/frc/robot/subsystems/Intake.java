// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final CANSparkMax intakeMotor;
  public Object intake;

  public Intake() {

    intakeMotor = new CANSparkMax(Constants.MotorConstants.intakeid, MotorType.kBrushless);

    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setInverted(Constants.MotorConstants.intakeInverted);
    intakeMotor.setIdleMode(IdleMode.kBrake);

  }

  public void intake(){
    intakeMotor.set(Constants.MotorConstants.intakeSpeed);
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }

  public void extake(){
    intakeMotor.set(-(Constants.MotorConstants.intakeSpeed));
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
