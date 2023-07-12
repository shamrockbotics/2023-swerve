// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {

  Drivetrain drivetrain;
  boolean fieldRelative;
  XboxController m_driveController;
  
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);



  /** Creates a new SwerveDrive. */
  public SwerveDrive(Drivetrain drivetrain, XboxController m_driverController, boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;

    this.fieldRelative = fieldRelative;
    this.m_driveController = m_driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getLeftY(), 0.1))
            * Constants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getLeftX(), 0.1))
            * Constants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRightX(), 0.1))
            * Constants.kMaxAngularSpeed;


    drivetrain.drive(xSpeed, ySpeed, rot, false);
    SmartDashboard.putNumber("number", xSpeed);
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
