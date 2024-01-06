// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class HoldWrist extends CommandBase {
  /** Creates a new HoldWrist. */
  Wrist wrist;
  double targetPosition;

  public HoldWrist(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = wrist.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.2;
    double error = targetPosition - wrist.getAngle();

    double output = error * kP;

    output = Math.copySign(Math.min(0.1, Math.abs(output)), output);
    if (Math.abs(error) < 0.01) output = 0;

    wrist.rotateWrist(output);
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
