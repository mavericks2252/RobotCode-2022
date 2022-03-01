// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallRun extends SubsystemBase {

  public DigitalInput topBallSensor; // need to create a new instance of this sensor
  public DigitalInput bottomBallSensor;
  CANSparkMax ballRunMotor;



  /** Creates a new Hopper. */
  public BallRun() { //Rename BallRun

    // Creates new spark max motor named feedWheel    
    ballRunMotor = new CANSparkMax(Constants.ball_run_motor_port, MotorType.kBrushless);
    ballRunMotor.setInverted(true);

    // Creates top and bottom sensors to detect balls in the ball run
    topBallSensor = new DigitalInput(Constants.top_ball_sensor_port);// set ballSensor equal to new instance of a digital input
    bottomBallSensor = new DigitalInput(Constants.bottom_ball_sensor_port);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("TopBallSensor State", getBallSensor(topBallSensor));
    SmartDashboard.putBoolean("BottomBallSensor State", getBallSensor(bottomBallSensor));
  }

  //Method to set PercentOutput of Ballrun Motor
  public void ballrunPercentOutput(double brSpeed){

    ballRunMotor.set(brSpeed);
    
    SmartDashboard.putNumber("BallRunSpeed", ballRunMotor.get());

  }
  //Method to stop BallRun Motor
  public void stopBallrun(){

    ballRunMotor.stopMotor();

  }

  //Method to get ballSensor 
  public boolean getBallSensor(DigitalInput sensor){

   return sensor.get();
    
  }



  }
