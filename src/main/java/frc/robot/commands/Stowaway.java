// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class Stowaway extends CommandBase {
  /** Creates a new Stowaway. */
  Elevator elevator;
  Wrist wrist;

  boolean isWristDone = false;
  boolean isElevDone = false;

  public Stowaway(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
    

    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isElevDone = false;
    isWristDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (elevator.getPosition() >= 5){
      elevator.retract(0.5);
    } else {
      isElevDone = true;
      elevator.stopElev();
    }

    if (wrist.getAngle() >= 3){
      wrist.rotateWrist(-Constants.MotorConstants.wristMotorSpeed);
    } else {
      isWristDone = true;
      wrist.stopWrist();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopWrist();
    elevator.stopElev();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isWristDone && isElevDone);
  }
}
