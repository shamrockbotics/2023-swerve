// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ExtendPosition extends CommandBase {
  Elevator elevator;
  double targetPosition;
  boolean finished = false;


  public ExtendPosition(Elevator elevator, double newPosition) {
    targetPosition = newPosition;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.5;
    double error = targetPosition - elevator.getPosition();

    double output = error * kP;

    output = Math.copySign(Math.min(0.7, Math.abs(output)), output);
        if (Math.abs(error) < 0.4){
            output = 0; 
            finished = true;
        } 

    
    elevator.extend(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElev();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
