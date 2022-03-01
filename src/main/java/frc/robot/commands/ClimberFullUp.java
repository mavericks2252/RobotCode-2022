// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberFullUp extends CommandBase {
  Climber climber;
  /** Creates a new ClimberPositionalControl. */
  public ClimberFullUp(Climber c) {
    climber = c;
  addRequirements(climber); //declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climber.setClimberPosition(Constants.climber_max_height);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    climber.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    
    if (climber.getClimberPositionInches()<Constants.climber_max_height){
      return false;
      
    }
    else{
      return true;
    }
  }
}
