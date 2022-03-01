// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class StartShooter extends CommandBase {
  /** Creates a new StartShooter. */
    Shooter shooter;

  public StartShooter(Shooter s) {
    shooter = s;
    addRequirements(shooter);//declare subsystem dependencies.
    SmartDashboard.putBoolean("Shooter Status", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("Shooter Status", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    shooter.setShooterVelocity(Constants.shooter_target_RPMs);
    
    //shooter.shooterMasterPercentOutput(Constants.shooterSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stop();
    SmartDashboard.putBoolean("Shooter Status", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
