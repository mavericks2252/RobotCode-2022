// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallRun;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
Intake intake;
BallRun ballRun;
Timer timer;

  /** Creates a new RunIntake. */
  public RunIntake(Intake i, BallRun bR) {
  intake = i;
  ballRun = bR;
  timer = new Timer();
  addRequirements(intake);  //declare subsystem dependencies.
  SmartDashboard.putBoolean("Intake Status", false); 
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("Intake Status", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.intakeSpeed(Constants.intakeSpeed);
    intake.extendIntake();
    ballRun.ballrunPercentOutput(Constants.ball_run_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    intake.retractIntake();
    SmartDashboard.putBoolean("Intake Status", false);
    timer.reset();
    timer.start();
    while (timer.get() < .1){
    ballRun.ballrunPercentOutput(Constants.ball_run_reverse_speed);
    }
    
    intake.stop();
    ballRun.stopBallrun();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if (ballRun.getBallSensor(ballRun.topBallSensor) == false 
        && ballRun.getBallSensor(ballRun.bottomBallSensor) == false)
    {
      
      timer.reset();
      timer.start();
      while (timer.get() < .05){}

      return true;

    }

    else {

      return false;

    }

  }
}
