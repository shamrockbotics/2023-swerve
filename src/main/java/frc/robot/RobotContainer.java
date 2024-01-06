// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExtendPosition;
import frc.robot.commands.HoldElevator;
import frc.robot.commands.HoldWrist;
import frc.robot.commands.SetWristPositon;
import frc.robot.commands.StopStayModules;
import frc.robot.commands.Stowaway;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.*;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);
  //private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(gyro);
  private final Intake intakeSubsystem = new Intake();
  private final Elevator elevatorSubsystem = new Elevator();
  private final Wrist wristSubsystem = new Wrist();
  private final LEDs ledsSubsystem = new LEDs();


  private final XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(Constants.OperatorConstants.OperatorControllerPort);


  ///Commands
  //Swerve Commands
  private final InstantCommand zeroSwerveHeading = new InstantCommand(() -> swerveSubsystem.zeroHeading());
  private final InstantCommand calibrateSwerveHeading = new InstantCommand(() -> swerveSubsystem.resetGyro());
  private final InstantCommand changeFieldOrientated = new InstantCommand(() -> swerveSubsystem.changeFieldOrientated(), swerveSubsystem);
  private final StopStayModules stayStopModules = new StopStayModules(swerveSubsystem);

  //Intake Commands
  private final Command intake = new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem);
  private final Command extake = new RunCommand(() -> intakeSubsystem.extake(), intakeSubsystem);
  private final Command stopIntake = new RunCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem);

  //Elevator Commands
  private final Command extend = new RunCommand(() -> elevatorSubsystem.extend(), elevatorSubsystem);
  private final Command retract = new RunCommand(() -> elevatorSubsystem.retract(operatorController.getRightTriggerAxis()), elevatorSubsystem);
  private final Command stopElevator = new RunCommand(() -> elevatorSubsystem.stopElev(), elevatorSubsystem);
  private final Command holdElevator = new HoldElevator(elevatorSubsystem);
  

  //Wrist Commands
  private final Command rotateWrist = new RunCommand(() -> wristSubsystem.rotateWrist(operatorController.getRightY()), wristSubsystem);
  private final Command rotateWristForward = new RunCommand(() -> wristSubsystem.rotateWrist(Constants.MotorConstants.wristMotorSpeed), wristSubsystem);
  private final Command rotateWristBackward = new RunCommand(() -> wristSubsystem.rotateWrist(-Constants.MotorConstants.wristMotorSpeed), wristSubsystem);
  private final Command stopWrist = new RunCommand(() -> wristSubsystem.stopWrist(), wristSubsystem);
  private final Command holdWrist = new HoldWrist(wristSubsystem);

  //LED Commands

  //Preset Scoring Commands

  private final Command scoreHigh = (
    new ExtendPosition(elevatorSubsystem, Constants.MotorConstants.scoreHighElev))
    .alongWith(new SetWristPositon(wristSubsystem, Constants.MotorConstants.scoreHighWrist));

  private final Command scoreMiddle = (
    new ExtendPosition(elevatorSubsystem, Constants.MotorConstants.scoreMidElev))
    .alongWith(new SetWristPositon(wristSubsystem, Constants.MotorConstants.scoreMidWrist));

  private final Command scoreGround = (
    new ExtendPosition(elevatorSubsystem, Constants.MotorConstants.scoreGroundElev))
    .alongWith(new SetWristPositon(wristSubsystem, Constants.MotorConstants.scoreGroundWrist));

  private final Command humanPlayerStation = (
    new ExtendPosition(elevatorSubsystem, Constants.MotorConstants.humanElev))
    .alongWith(new SetWristPositon(wristSubsystem, Constants.MotorConstants.humanWrist));

  private final Command Stowaway = new Stowaway(elevatorSubsystem, wristSubsystem);


  //Auto Commands

  private final ExtendPosition ExtendElevHigh = new ExtendPosition(elevatorSubsystem, Constants.AutoConstants.ElevHigh);

  //Swerve Trajectory Creation

  // 1. Create trajectory settings
  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

  // 2. Generate trajectory
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      /*new Translation2d(2, 1),
      new Translation2d(4, -1),
      new Translation2d(6, 0),
      new Translation2d(4, 1),
      new Translation2d(2, -1)*/

    ),
    new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

  // 3. Define PID Controllers for Tracking Trajectory
  PIDController xController = new PIDController(AutoConstants.kPYController, 0, 0);
  PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, 
    Constants.AutoConstants.kThetaControllerConstraints);
  
  

  //4. Construct command to follow trajectory
  SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    xController,
    yController, 
    thetaController,
    swerveSubsystem::setModuleStates,
    swerveSubsystem);

 

  // 5. Add some init and wrap-up, and return everything
  Command moveBack = 
    (new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())))
      .andThen(new InstantCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem))
      .andThen(swerveControllerCommand)
      .andThen(new StopStayModules(swerveSubsystem));




  SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_chooser.setDefaultOption("Default Auton", moveBack);
    m_chooser.addOption("Move Back", moveBack);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);




    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -driverController.getLeftY(), 
      () -> driverController.getLeftX(), 
      () -> driverController.getRightX(), 
      () -> swerveSubsystem.getIsFieldOrientated()));
    
    intakeSubsystem.setDefaultCommand(stopIntake);
    elevatorSubsystem.setDefaultCommand(stopElevator);
    wristSubsystem.setDefaultCommand(stopWrist);

  




    configureBindings();
  }

  private void configureBindings() {
    //new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(zeroSwerveHeading);

    new JoystickButton(driverController, 4).onTrue(changeFieldOrientated);

    new JoystickButton(driverController, 1).onTrue(zeroSwerveHeading);

    new JoystickButton(driverController, 2).whileTrue(stayStopModules);

    //new JoystickButton(operatorController, Constants.OperatorConstants.highScoreButton).whileTrue(scoreHigh);

    //new JoystickButton(operatorController, Constants.OperatorConstants.highScoreButton).whileTrue(Stowaway);
    
    //new JoystickButton(operatorController, Constants.OperatorConstants.middleScoreButton).whileTrue(scoreMiddle);

    //new JoystickButton(operatorController, Constants.OperatorConstants.groundScoreButton).whileTrue(scoreGround);

    //new JoystickButton(operatorController, Constants.OperatorConstants.playerStationButton).whileTrue(humanPlayerStation);

    new JoystickButton(operatorController, Constants.OperatorConstants.intakeButton).whileTrue(intake);

    new JoystickButton(operatorController, Constants.OperatorConstants.extakeButton).whileTrue(extake);

    new JoystickButton(operatorController, Constants.OperatorConstants.extendButton).whileTrue(extend);

    new JoystickButton(operatorController, Constants.OperatorConstants.retractButton).whileTrue(retract);

    new JoystickButton(operatorController, 4).whileTrue(rotateWristForward);

    new JoystickButton(operatorController, 3).whileTrue(rotateWristBackward);

    //new JoystickButton(operatorController, Constants.OperatorConstants.wristButton).whileTrue(rotateWrist);

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = m_chooser.getSelected();
    

    return autoCommand;


    //return null;

    //return swerveForward;
  }
}