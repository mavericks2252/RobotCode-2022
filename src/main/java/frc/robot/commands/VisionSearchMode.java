// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class VisionSearchMode extends CommandBase {
  Turret turret;
  Vision vision;
  /** Creates a new VisionSearchMode. */
  public VisionSearchMode(Turret t, Vision v) {
    turret = t;
    vision = v;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(turret.getTurretPosition() < 179){

    turret.turretSpeed(Constants.turret_search_speed);
    }
    else{
      turret.setTurretPosition(-179);
      
      while (turret.getTurretPosition()> -179 && vision.limeLightTargetCheck() != 1){

      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    turret.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(vision.limeLightTargetCheck() == 1){

    return true;
    }
    else {
      return false;
    }
  }
}
