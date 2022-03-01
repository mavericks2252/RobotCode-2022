// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    /*  Consider Using Classes for Constants of each Subsystem to limit the imports of constants
    public final class DriveConstants{ 
        Put DriveTrain Constants here
    }
    public final class ShooterConstants{}
    public final class IntakeConstants{}
    public final class VisionConstnats{}
    public final class TurretConstants{}
    public final class XboxConstants{}
    public final class AutoConstants{}
    */
    //Drive Base 
    
    public static final int left_motor_slave_port = 1;
    public static final int left_motor_master_port = 4;
    public static final int right_motor_slave_port = 3;
    public static final int right_motor_master_port = 2;
    public static final int pigeon_port = 12;

    //Shooter
    public static final int shooter_motor_master_port = 5;
    public static final int shooter_motor_slave_port = 6;
    public static final int accelerator_wheel_motor_port = 7;
    public static final int shooter_solenoid_port = 0;
    public static final double shooter_target_RPMs = 2700;
 
    public static final double ball_run_reverse_speed = -.25;
    public static final int ball_run_motor_port = 10;
    public static final int top_ball_sensor_port = 8;
    public static final int bottom_ball_sensor_port = 6;
    public static final double ball_run_speed = 1;

    //Climber
    public static final int climber_motor_port = 10;
    public static final double climber_percent_output = 0;
    public static final double climber_drum_circumference = Math.PI * 1.25;
    public static final double climber_gearbox_ratio = 22.67;
    public static final double climber_max_height = 23;
    public static final double climber_min_hight = -1;
    public static final double climber_motor_speed = 0;
    public static final double climber_kF = .3;
    public static final double climber_kP = 0;
    public static final double climber_kI = 0;
    public static final double climber_kD = 0;
    public static final int climber_home_sensor_port = 5;
    public static final double climber_down_full_speed = -1;
    public static final double climber_down_slow_speed = -0.30;

    //Intake
    public static final int intake_motor_port = 8;
    public static final double intakeSpeed = 1;


    //Turret
    public static final int turret_motor_port = 9;
    public static final double turret_kP = 0.09;
    public static final double turret_kD = 0;
    public static final double turret_kI = 0;
    public static final double minimum_turn_command = 0.25;
    public static final double turret_gear_ratio = 0;
    public static final double turret_rotation_speed = 0;
    
    //SHOOTER PID Values
    public static final double shooter_kF = (.60 * 1023) /11238;
    public static final double shooter_kD = 0;
    public static final double shooter_kI = 0.0000001;
    public static final double shooter_kP = 0.0899999142;
    public static final double shooter_IZone = 100;
    public static final double shooter_Ramp_Time = 0.1;
    public static final int shooter_timeout_ms = 30;
    

    //Shooter Constants
    public static final double shooter_speed = .45;


    //Drivetrain
    public static final double deadBand = .1;
    public static final double manual_drive_speed = .1;

    public static final double Falcon_Ticks_Per_Rev = 2048;
    public static final SupplyCurrentLimitConfiguration drive_motor_current_limit = new SupplyCurrentLimitConfiguration(true, 30, 30, .1);
    public static final StatorCurrentLimitConfiguration drive_motor_stator_current_limit = new StatorCurrentLimitConfiguration(true , 30 , 40 , .5);


//xbox controller buttons

    //ControllerPorts
    public static final int operator_controller_port = 0;
    public static final int driver_controller_port = 1;

    //Joysticks
    public static final int lStickYAxis = 1;
    public static final int rStickXAxis = 4;
    public static final int lStickXAxis = 0;
    public static final int rStickYAxis = 5;

    //Buttons
    public static final int xButton = 3;
    public static final int aButton = 1;
    public static final int bButton = 2;
    public static final int yButton = 4;

    //Triggers
    public static final int left_trigger = 2;
    public static final int right_trigger = 3;

    //Bumpers
    public static final int left_bumper = 5;
    public static final int right_bumper = 6;
	


    //Pneumatics
    public static final int pdh_port = 15;
    public static final int pneumatic_hub_port = 1;
    public static final int climber_solenoid_channel = 1;
    public static final int intake_solenoid_channel = 2;
    public static final int shooter_solenoid_channel = 3;
    public static final int hr_solenoid_channel = 4;
    

    //Auto
    public static final double ks_volts = 0.71537;
    public static final double kv_volts_seconds_per_meter = 2.4184;
    public static final double ka_volts_seconds_squared_per_meter = 0.38952;

    public static final double kp_drive_velocity = 3.416;

    public static final double k_track_width_meters = 0.6731; //26.5 inches
    public static final DifferentialDriveKinematics k_drive_kinematics = new DifferentialDriveKinematics(k_track_width_meters);
    
    public static final double k_max_speed_meters_per_second = 2;
    public static final double k_max_acceleration_meters_per_second_squared = 2;

    public static final double k_ramesete_b = 2;
    public static final double k_ramesete_zeta = 0.7;
    public static final int dt_gearbox_ratio = 7;
    public static final double dt_wheel_dia_meters = .09906;
    public static final double dt_wheel_circumference_meters = dt_wheel_dia_meters * Math.PI;
    public static final double drivetrain_max_voltage = 9.5;
    public static final double max_reverse_angle = 0;
    public static final double max_forward_angle = 0;
    public static final double turret_search_speed = 0;
    
    
    
    
   
    
   
   
    



}
