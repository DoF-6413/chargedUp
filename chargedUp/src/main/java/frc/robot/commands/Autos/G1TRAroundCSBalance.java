// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class G1TRAroundCSBalance extends SequentialCommandGroup {

  PathPlannerTrajectory kAroundCSLeftBalance = PathPlanner.loadPath("AroundCSLeftBalance", new PathConstraints(1, .5));
  
  /** Creates a new G1TRAroundCSBalance. */
  public G1TRAroundCSBalance(ArmPIDSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endEffector, GyroSubsystem gyro, PoseEstimator pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreConehigh(arm, telescoper, endEffector, drive),
      new TrajectoryRunner(drive, pose, ()->kAroundCSLeftBalance.relativeTo(pose.getcurrentPose()), true),
      new gyroBalance(gyro, drive)
    );
  }
}
