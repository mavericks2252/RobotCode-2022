// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class VisionTracking extends CommandBase {
  Vision vision;
  DriveTrain driveTrain;
  
  private double tv;
  private double headingError;
  private double steeringAdjust;

  private double leftSpeed;
  private double rightSpeed;

  /** Creates a new VisionTracking. */
  public VisionTracking(Vision v, DriveTrain dt) {
    vision = v;
    driveTrain = dt;
  
    // This is going to need to intake the Turret and not the drivetrain
    addRequirements(vision, driveTrain); //declare subsystem dependencies.
  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = vision.limeLightTargetCheck();

    if(tv == 1) {
      headingError =  vision.limeLightHorizontalPosition();
      SmartDashboard.putNumber("Mode", 1);
      steeringAdjust = (headingError * Constants.turret_kP);

      if (Math.abs(headingError) > .25){

        steeringAdjust = (headingError * Constants.turret_kP);
          if (Math.abs(steeringAdjust) < Constants.minimum_turn_command){

            steeringAdjust = (Constants.minimum_turn_command * headingError);

          }
      }

      else {
        steeringAdjust = 0;
      }

      leftSpeed = -steeringAdjust;
      rightSpeed = steeringAdjust;

      driveTrain.ManualTurn(leftSpeed, rightSpeed);
      
      SmartDashboard.putNumber("leftSpeed", leftSpeed);
      SmartDashboard.putNumber("rightSpeed", rightSpeed);
     }

     else {

      SmartDashboard.putNumber("Mode", 2);

     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
