// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallRun;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  
  /** Creates a new TestAuto. */
  public TestAuto(Trajectory trajectory, Trajectory trajectory2, DriveTrain driveTrain, Intake intake, BallRun ballRun) {
    
    ParallelDeadlineGroup shootAndWait = new ParallelDeadlineGroup(new WaitCommand(3), new StartShooter(RobotContainer.shooter));
    ParallelDeadlineGroup shootAndWait2 = new ParallelDeadlineGroup(new WaitCommand(3), new StartShooter(RobotContainer.shooter));
   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      
      shootAndWait,

      new FollowPathAndIntake(trajectory, driveTrain, intake, ballRun), 

      shootAndWait2,

      new FollowPathAndIntake(trajectory2, driveTrain, intake, ballRun)
      
    
      );
  }
}
