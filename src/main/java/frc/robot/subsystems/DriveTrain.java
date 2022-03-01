// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.commands.ManualDrive;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
WPI_TalonFX rightDriveMotorMaster;
WPI_TalonFX rightDriveMotorSlave;
WPI_TalonFX leftDriveMotorMaster;
WPI_TalonFX leftDriveMotorSlave;
DifferentialDrive drive;
WPI_Pigeon2 pigeon;


private final DifferentialDriveOdometry m_Odometry;

  public DriveTrain(){
    // Creates new left and right slave and master drivetrain motors
    rightDriveMotorMaster = new WPI_TalonFX(Constants.right_motor_master_port);
    rightDriveMotorSlave = new WPI_TalonFX(Constants.right_motor_slave_port);    
    leftDriveMotorMaster = new WPI_TalonFX(Constants.left_motor_master_port);
    leftDriveMotorSlave = new WPI_TalonFX(Constants.left_motor_slave_port);
    // Creates a new instance of differential drive
    drive = new DifferentialDrive(leftDriveMotorMaster, rightDriveMotorMaster);
    
    //Left Motors
      // Sets motor and encoder defaults
    leftDriveMotorMaster.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorMaster.setInverted(true);
    leftDriveMotorSlave.setInverted(true);
    leftDriveMotorSlave.follow(leftDriveMotorMaster);
    leftDriveMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftDriveMotorMaster.setSelectedSensorPosition(0);

   

   //Right Motors
      // Sets motor and encoder defaults
    rightDriveMotorMaster.setNeutralMode(NeutralMode.Brake);
    rightDriveMotorMaster.setInverted(false);
    rightDriveMotorSlave.setInverted(false);
    rightDriveMotorSlave.follow(rightDriveMotorMaster);
    rightDriveMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightDriveMotorMaster.setSelectedSensorPosition(0);
    
    //Current Limits
    rightDriveMotorMaster.configSupplyCurrentLimit(Constants.drive_motor_current_limit);
    rightDriveMotorSlave.configSupplyCurrentLimit(Constants.drive_motor_current_limit);
    leftDriveMotorMaster.configSupplyCurrentLimit(Constants.drive_motor_current_limit);
    leftDriveMotorSlave.configSupplyCurrentLimit(Constants.drive_motor_current_limit);
    
    rightDriveMotorMaster.configStatorCurrentLimit(Constants.drive_motor_stator_current_limit);
    rightDriveMotorSlave.configStatorCurrentLimit(Constants.drive_motor_stator_current_limit);
    leftDriveMotorMaster.configStatorCurrentLimit(Constants.drive_motor_stator_current_limit);
    leftDriveMotorSlave.configStatorCurrentLimit(Constants.drive_motor_stator_current_limit);
    // Creates new pigeon2 gyro
    pigeon = new WPI_Pigeon2(Constants.pigeon_port);
   
    m_Odometry = new DifferentialDriveOdometry(pigeon.getRotation2d());
    


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      setDefaultCommand(RobotContainer.manualDrive); //Not the right place for this... Put it in RobotContainer
      m_Odometry.update(pigeon.getRotation2d(),
       ticksToMeters(leftDriveMotorMaster.getSelectedSensorPosition()),
       ticksToMeters(rightDriveMotorMaster.getSelectedSensorPosition()));

       SmartDashboard.putNumber("RightMasterAmps", RobotContainer.pdh.getCurrent(0));
       SmartDashboard.putNumber("RightSlaveAmps", RobotContainer.pdh.getCurrent(1));
       SmartDashboard.putNumber("LeftMasterAmps", RobotContainer.pdh.getCurrent(18));
       SmartDashboard.putNumber("LeftSlaveAmps", RobotContainer.pdh.getCurrent(19));
  }

  // Creates joystick deadband
  public double deadBand(int axis){

    if(Math.abs(RobotContainer.driverController.getRawAxis(axis)) <= Constants.deadBand){
      return 0;
    }
    else{
      return RobotContainer.driverController.getRawAxis(axis);
    }
  }

  // Creates arcade drive to manually drive the robot
  public void ManualDrive(XboxController controller, double speed){

    drive.arcadeDrive( -deadBand(Constants.lStickYAxis), deadBand(Constants.rStickXAxis));

  }

  public void ManualTurn(double leftSpeed, double rightSpeed){

    drive.arcadeDrive(leftSpeed, rightSpeed);

  }

  public double ticksToMeters(double position){

    return position / Constants.Falcon_Ticks_Per_Rev / Constants.dt_gearbox_ratio * Constants.dt_wheel_circumference_meters;

  }

  public double velocityToMetersPerSecond(double velocity){

    return ticksToMeters(velocity) * 10; //10 100 milisecond units per second

  }

  public void resetOdemetry(Pose2d pose) {
    resetEncoders();
    m_Odometry.resetPosition(pose, pigeon.getRotation2d());

    SmartDashboard.putBoolean("pose reset", true);
    }

  public void resetEncoders(){

      leftDriveMotorMaster.setSelectedSensorPosition(0);
      rightDriveMotorMaster.setSelectedSensorPosition(0);

    }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){

    return new DifferentialDriveWheelSpeeds(
        velocityToMetersPerSecond(leftDriveMotorMaster.getSelectedSensorVelocity()),   //get Left Drive Motor Speed
        velocityToMetersPerSecond(rightDriveMotorMaster.getSelectedSensorVelocity())); //get Right Drive Motor Speed

  }

  public double getHeading(){

    return pigeon.getRotation2d().getDegrees();

  }

  public Pose2d getPose2d(){

    return m_Odometry.getPoseMeters();

  }

  public void tankDriveVolts(double leftVolts, double rightVolts){

    leftDriveMotorMaster.setVoltage(leftVolts);
    rightDriveMotorMaster.setVoltage(rightVolts);
    drive.feed();
  }

  public void stop(){

    rightDriveMotorMaster.stopMotor();
    leftDriveMotorMaster.stopMotor();

  }

}
