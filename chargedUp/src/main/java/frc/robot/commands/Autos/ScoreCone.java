// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCone extends SequentialCommandGroup {

  /** Creates a new ScoreCone. */
  public ScoreCone(ArmSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEfctr, DrivetrainSubsystem drive) {
    // PathPlannerTrajectory m_backUpRed = PathPlanner.loadPath("BackUpRed", new PathConstraints(2, 0.45));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telescoper),
      new RotationPID(arm, -100),
      new TelescoperPID(telescoper, 145),
      new RotationPID(arm, -83),
      new ParallelCommandGroup(
        new EndEffectorRunner(NEfctr, -0.8, 3),
        new TelescoperPID(telescoper, 1)
        ),
      new RotationPID(arm, 0)
      // new TrajectoryRunner(drive, m_backUpRed.relativeTo(drive.getPose()), true)
    );
  }
}
