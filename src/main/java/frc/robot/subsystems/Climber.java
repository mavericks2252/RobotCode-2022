// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {

  WPI_TalonFX climberMotor;
  Solenoid climberSolenoid;
  Solenoid hrSolenoid;
  public DigitalInput climberHomeSensor;
  /** Creates a new Climber. */
  public Climber() {
    // Creates new talon fx called climberMotor
    climberMotor = new WPI_TalonFX(Constants.climber_motor_port);
    climberMotor.configFactoryDefault();
    climberMotor.configNeutralDeadband(Constants.deadBand);
    
    // Configure sensor and set defaults
    climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberMotor.setSelectedSensorPosition(0);
    climberMotor.setInverted(TalonFXInvertType.CounterClockwise);
    climberMotor.setNeutralMode(NeutralMode.Brake);

    // Configure and enable soft limits
    climberMotor.configForwardSoftLimitThreshold(inchesToTicks(Constants.climber_max_height));
    //climberMotor.configReverseSoftLimitThreshold(inchesToTicks(Constants.climber_min_hight));
    climberMotor.configForwardSoftLimitEnable(true);
    //climberMotor.configReverseSoftLimitEnable(true);
    climberHomeSensor = new DigitalInput(Constants.climber_home_sensor_port);
    

    // Configure PID 
    climberMotor.config_kF(0, Constants.climber_kF, 0);
    climberMotor.config_kP(0, Constants.climber_kP, 0);    
    climberMotor.config_kI(0, Constants.climber_kI, 0);
    climberMotor.config_kD(0, Constants.climber_kD, 0);

    

    // Creates solenoids and sets defaults
    climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.climber_solenoid_channel);
    hrSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.hr_solenoid_channel);
    climberSolenoid.set(false);
    hrSolenoid.set(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberHeight", getClimberPositionInches());
    
    if(climberHomeSensor.get() == false){

      climberMotor.setSelectedSensorPosition(0);
    }
  }


  public void manualControl(int axis){
    double speed;
    if(climberHomeSensor.get() == false && RobotContainer.opereratorController.getRawAxis(axis) > 0){
       speed = 0;
    }

    else{

       speed = RobotContainer.opereratorController.getRawAxis(axis);

    }
    SmartDashboard.putNumber("Speed", speed);
    climberMotor.set(ControlMode.PercentOutput,  -speed);
  }

  public void climberPercentOutput(Double cSpeed){

    climberMotor.set(ControlMode.PercentOutput, cSpeed);
  }

  // TicksToInches method.... Convert ticks to inches
  public double ticksToInches(double ticks){

    return ticks / Constants.Falcon_Ticks_Per_Rev / Constants.climber_gearbox_ratio * Constants.climber_drum_circumference; 

  }

  // Set Climber Position in Inches Take in inches and run that number through TicksToInches and set position of climber motor
      // Unit Conversion  Inches = Inches / Circumference of drum * GearRatio * TicksPerRevFalcon
  public double inchesToTicks(double inches){

    return inches / Constants.climber_drum_circumference * Constants.climber_gearbox_ratio * Constants.Falcon_Ticks_Per_Rev;

  }
     
  // Method to set position
  public void setClimberPosition(double height){

    climberMotor.set(ControlMode.Position, inchesToTicks(height));

  }

  public double getClimberPositionInches(){

    return ticksToInches(climberMotor.getSelectedSensorPosition());
  }

  // Method to release static hooks
  public void staticHookRelease(){   

    hrSolenoid.set(true);

  }

  public void stop(){

    climberMotor.stopMotor();

  }

  public void articulateClimber(){

    climberSolenoid.toggle();

    SmartDashboard.putBoolean("climberUp=True", climberSolenoid.get());

  }

 
    
}
