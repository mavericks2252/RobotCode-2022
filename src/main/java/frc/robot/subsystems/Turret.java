// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  
  WPI_TalonFX turretMotor;
  
  
  
  /** Creates a new Turret. */
  public Turret() {

    turretMotor = new WPI_TalonFX(Constants.turret_motor_port);
    turretMotor.setInverted(false); //incase we need to reverse the direction of the motor
    // Set FeebackDevice and configure default settings
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    turretMotor.setSelectedSensorPosition(0);
    
    // Set forward Limit
    turretMotor.configForwardSoftLimitThreshold(degreesToTicks(Constants.max_forward_angle));
    // Set Reverse Limit
    turretMotor.configReverseSoftLimitThreshold(degreesToTicks(Constants.max_reverse_angle));

    // Enable forward soft limit
    turretMotor.configForwardSoftLimitEnable(true); 
    // Enable reverse soft limit
    turretMotor.configReverseSoftLimitEnable(true);  
    
    //Configure Kp, Ki, Kd of the turret
    turretMotor.config_kP(0, Constants.turret_kP);
    turretMotor.config_kI(0, Constants.turret_kI);
    turretMotor.config_kD(0, Constants.turret_kD);

   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTurretPosition(){

    return ticksToDegrees(turretMotor.getSelectedSensorPosition());
  }

  //Set PercentOutput Method take in a speed argument
  public void turretSpeed(double tSpeed){

    turretMotor.set(ControlMode.PercentOutput, tSpeed);

  }

  // Reset Encoder Method (we may need to take in a position to set the turret at dependant upon which auto we are running)
  public void resetTurretEncoder(){

    turretMotor.setSelectedSensorPosition(0);

  }

  // ticksToDegrees Method Take in Ticks Return Degrees
      // ticks / ticksPerRevFalcon / gear Ratio * 360 (I think this math is right)  Return this number
  public double ticksToDegrees(double ticks){

    return ticks / Constants.Falcon_Ticks_Per_Rev / Constants.turret_gear_ratio * 360;

  }
  
  // DegreesToTicks Method
      // Degrees / 360 * GearRatio * TicksPerRevFalcon (I think this math is right)  Return this number
  public double degreesToTicks(double degrees){

    return degrees / 360 * Constants.turret_gear_ratio * Constants.Falcon_Ticks_Per_Rev;

  }

  // We may need a set Position Method (we will need this so if soft limit is hit then then we can set the potion all the way around to other side)
      // Will need to take in degrees and run through degreesToTicks then set the position to that number
  public void setTurretPosition(double degrees){

    turretMotor.set(ControlMode.Position, degreesToTicks(degrees));

  }
  
  public void stop(){

    turretMotor.stopMotor();

  }

 

}
