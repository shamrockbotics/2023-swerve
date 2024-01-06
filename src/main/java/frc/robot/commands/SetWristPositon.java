// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWristPositon extends CommandBase {

  Wrist wrist;
  double targetAngle;
  boolean finished = false;
  /** Creates a new SetWristPositon. */
  public SetWristPositon(Wrist wrist, double newAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    targetAngle = newAngle;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.05;
    double error = wrist.getAngle() - targetAngle;

    double output = error * kP;

    output = Math.copySign(Math.min(0.3, Math.abs(output)), output);
        
    if (Math.abs(error) < 0.2){
      output = 0;
      finished = true;
    }

    
    wrist.rotateWrist(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
