// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberFullDown extends CommandBase {
  Climber climber;
  /** Creates a new ClimberFullDown. */
  public ClimberFullDown(Climber c) {
    climber = c;
    addRequirements(climber); // declare subsytem dependencies
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(climber.getClimberPositionInches() > 3){

      climber.climberPercentOutput(Constants.climber_down_full_speed);

      SmartDashboard.putBoolean("Climber Full Speed", true);
    }
    else{

      climber.climberPercentOutput(Constants.climber_down_slow_speed);

      SmartDashboard.putBoolean("Climber Full Speed", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;

    if (climber.climberHomeSensor.get() == false){
      return true;

      
    }
    else{
      return false;
    }
  }
}
