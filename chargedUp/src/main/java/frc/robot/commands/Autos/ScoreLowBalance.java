// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.gyroBalance;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLowBalance extends SequentialCommandGroup {
  /** Creates a new ScoreMidBalance. */

  PathPlannerTrajectory kBackToBalance = PathPlanner.loadPath("BackToBalance", new PathConstraints(1.5, 1.2));
 

  public ScoreLowBalance(ArmPIDSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endEffector, GyroSubsystem gyro, PoseEstimator pose) {
   
    HashMap<String, Command> eventCenterLScoreOutBalanceMap = new HashMap<>();
    eventCenterLScoreOutBalanceMap.put("BackOutArm", new BackingOutArm(arm, telescoper, endEffector));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreConeLow(arm, telescoper, endEffector, drive),
      new FollowPathWithEvents(new TrajectoryRunner(drive, pose, ()->kBackToBalance.relativeTo(pose.getcurrentPose()), true),
      kBackToBalance.getMarkers(),
      eventCenterLScoreOutBalanceMap
      ),
      new ParallelCommandGroup(
        Commands.runOnce(
          () -> {
            arm.setGoal(Units.degreesToRadians(0)+ArmConstants.kArmOffsetRads);
            arm.enable();
          },
          arm),
          new WaitUntilCommand(()-> arm.atGoal()),
          new gyroBalance(gyro, drive)
      )
    );
  }
}
