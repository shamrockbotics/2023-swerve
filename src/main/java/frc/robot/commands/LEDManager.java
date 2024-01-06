// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;

public class LEDManager extends CommandBase {
  
  private LEDs led;
  int count = 0;
  int flagNum = 0;


  public LEDManager(final LEDs led) {
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDSAttached()){
      count++;
      if (count == 100){
        count = 0;
        led.cycleFlags(Constants.LEDConstants.flags, flagNum);
        flagNum++;
        if (flagNum > Constants.LEDConstants.flags.length){
          flagNum = 0;
        }
      }
    } // end of is robot enabled
    else {
      led.setAllOff();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
