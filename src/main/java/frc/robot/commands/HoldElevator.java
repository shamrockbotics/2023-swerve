// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class HoldElevator extends CommandBase {
  /** Creates a new HoldElevator. */
  Elevator elevator;
  double targetPosition;

  public HoldElevator(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = elevator.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.1;
    double error = targetPosition - elevator.getPosition();
    
    double output = error * kP;

    output = Math.copySign(Math.min(0.1, Math.abs(output)), output);
    if (Math.abs(error) < 0.01) output = 0;

    elevator.extend(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
