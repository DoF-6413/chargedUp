// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeRightGridLow extends SequentialCommandGroup {
  /** Creates a new command that scores a cone on the right grid low right (Blue Alliance) */
  public ScoreConeRightGridLow(ArmSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endEffector, PoseEstimator poseEstimator) {
    // Add your commands in the addCommands() call, e.g.
    PathPlannerTrajectory gridRightLow = PathPlanner.loadPath("gridRight", new PathConstraints(1, 1));
    // addCommands(new FooCommand(), new BarCommand());
    HashMap<String, Command> eventMapGridRightLow = new HashMap<>();
    eventMapGridRightLow.put("armout", new PositionLow(arm, telescoper));

    PathPlannerTrajectory gridRightLowTraj = PathPlanner.generatePath(
      new PathConstraints(1, 1),
      new PathPoint(gridRightLow.getInitialPose().getTranslation(), gridRightLow.getInitialPose().getRotation()),
      new PathPoint(new Translation2d(3.35,7), new Rotation2d(1)));

    addCommands(
      new TrajectoryRunner(drive, gridRightLow.relativeTo(poseEstimator.getcurrentPose()), false),

      new FollowPathWithEvents(
        new TrajectoryRunner(drive, gridRightLowTraj.relativeTo(poseEstimator.getcurrentPose()), false),
        gridRightLow.getMarkers(),
        eventMapGridRightLow
        ),
      
    );
  }
}
