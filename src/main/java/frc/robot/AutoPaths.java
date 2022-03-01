// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class AutoPaths {

String testJSON ="paths/output/test.wpilib.json";
  String test2JSON ="paths/output/Test2.wpilib.json";
  public static Trajectory testTrajectory = new Trajectory();
  public static Trajectory test2Trajectory = new Trajectory();

public void autoPaths(){

    
    try {
        Path testPath = Filesystem.getDeployDirectory().toPath().resolve(testJSON);
        testTrajectory = TrajectoryUtil.fromPathweaverJson(testPath);
      } catch (IOException ex){
        DriverStation.reportError("Unable to open Trajectory: " + testJSON, ex.getStackTrace());
      }
  
  
      
    try {
        Path test2Path = Filesystem.getDeployDirectory().toPath().resolve(test2JSON);
        test2Trajectory = TrajectoryUtil.fromPathweaverJson(test2Path);
      } catch (IOException ex){
        DriverStation.reportError("Unable to open Trajectory: " + test2JSON, ex.getStackTrace());
      }

}

}
