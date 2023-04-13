// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.gyroBalance;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TelescoperSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterLScoreOutBalance extends SequentialCommandGroup {

  
  
  PathPlannerTrajectory kOverCSBalance = PathPlanner.loadPath("OverCSBalance", new PathConstraints(1.5, 1.2));
  
  
  /** Creates a new CenterLScoreOutBalance. */
  public CenterLScoreOutBalance(ArmPIDSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endEffector, GyroSubsystem gyro, PoseEstimator pose) {
    HashMap<String, Command> eventCenterLScoreOutBalanceMap = new HashMap<>();
    eventCenterLScoreOutBalanceMap.put("BackOutArm", new BackingOutArm(arm, telescoper, endEffector));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreCone(arm, telescoper, endEffector, drive),
      new FollowPathWithEvents(new TrajectoryRunner(drive,pose, ()-> kOverCSBalance.relativeTo(pose.getcurrentPose()), true),
      kOverCSBalance.getMarkers(),
      eventCenterLScoreOutBalanceMap
      ),
      new gyroBalance(gyro, drive)
    );
  }
}
