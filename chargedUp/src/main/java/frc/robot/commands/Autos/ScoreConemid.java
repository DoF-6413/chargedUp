// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.ArmControls.EndEffectorRunner;

import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConemid extends SequentialCommandGroup {
  /** Creates a new ScoreCone. */
  public ScoreConemid(ArmPIDSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEfctr, DrivetrainSubsystem drive) {
    // PathPlannerTrajectory m_backUpRed = PathPlanner.loadPath("BackUpRed", new PathConstraints(2, 0.45));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telescoper),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(-ArmConstants.kHPMP )+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal()),
      new TelescoperPID(telescoper, TelescoperConstants.kMCGB),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(-ArmConstants.kMidScoreFinal)+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal()),
        
        //make the following a follow path with events to change time
        new ParallelCommandGroup(
            new EndEffectorRunner(NEfctr, -0.8, 0.5),
            new TelescoperPID(telescoper, 1)
            ),
          Commands.runOnce(
            () -> {
              arm.setGoal(Units.degreesToRadians(0)+ArmConstants.kArmOffsetRads);
              arm.enable();
            },
            arm),
            new WaitUntilCommand(()-> arm.atGoal())
    );
  }
}
