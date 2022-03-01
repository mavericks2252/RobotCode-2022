// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import javax.naming.LimitExceededException;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private double tv;
  private double tx;
  private double ty;  

  public Vision() {
   
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tv = limeLightTargetCheck();
    tx = limeLightHorizontalPosition();
    ty = limeLightVerticalPosition();
   
    SmartDashboard.putNumber("Vision Target Found", tv);
    SmartDashboard.putNumber("Horizontal Position", tx);
    SmartDashboard.putNumber("Vertical Position", ty);

  }

  public double limeLightTargetCheck(){

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

  }

  public double limeLightHorizontalPosition(){

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

  }

  public double limeLightVerticalPosition(){

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

  }

  // Calculate Distance Method See Me I can help with the Trig on this.

  

}
