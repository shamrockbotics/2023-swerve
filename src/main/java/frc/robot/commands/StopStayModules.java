// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class StopStayModules extends CommandBase {
  /** Creates a new StopStayModules. */

  private final SwerveSubsystem swerveSubsystem;
  public StopStayModules(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    


    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SwerveModuleState[] stopStates = {
      new SwerveModuleState(0.0, new Rotation2d(-0.75)), //front left
      new SwerveModuleState(0.0, new Rotation2d(0.75)), // front right
      new SwerveModuleState(0.0, new Rotation2d(0.75)), // back left 
      new SwerveModuleState(0.0, new Rotation2d(-0.75))}; // back right
    swerveSubsystem.setModuleStates(stopStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
